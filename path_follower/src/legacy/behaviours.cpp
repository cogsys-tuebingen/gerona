#include <path_follower/legacy/behaviours.h>
#include <path_follower/pathfollower.h>
#include <utils_general/MathHelper.h>
#include <path_follower/utils/path_exceptions.h>

using namespace Eigen;

//##### BEGIN Behaviour
Behaviour::Behaviour(PathFollower &parent):
    parent_(parent),
    controller_(dynamic_cast<RobotController_Legacy_Ackermann_Pid*>(parent.getController()))
{
    visualizer_ = Visualizer::getInstance();
}

const SubPath& Behaviour::getCurrentSubPath()
{
    //return parent_.paths_[index];
    return parent_.path_->getCurrentSubPath();
}
size_t Behaviour::getSubPathCount() const
{
    return parent_.path_->subPathCount();
}

double Behaviour::distanceTo(const Waypoint& wp)
{
    Eigen::Vector3d pose = parent_.getRobotPose();
    return hypot(pose(0) - wp.x, pose(1) - wp.y);
}

void Behaviour::setStatus(int status)
{
    *status_ptr_ = status;
}


Behaviour* Behaviour::initExecute(int *status)
{
    status_ptr_ = status;

    Behaviour* next_behaviour = selectNextWaypoint();
    return next_behaviour;

//    if(next_behaviour != this) {
//        return next_behaviour;
//    }
//    return this;
}

Path::Ptr Behaviour::getPath()
{
    return parent_.getPath();
}

PathFollowerParameters& Behaviour::getOptions()
{
    return parent_.opt_;
}
//END Behaviour



//##### BEGIN BehaviourOnLine

BehaviourOnLine::BehaviourOnLine(PathFollower& parent)
    : Behaviour(parent)
{
    controller_->initOnLine();
}


Behaviour* BehaviourOnLine::execute(int *status)
{
    Behaviour* next = initExecute(status);
    if(next != this) {
        return next;
    }

    controller_->setPath(getPath());
    controller_->behaveOnLine();

    return this;
}


Behaviour* BehaviourOnLine::selectNextWaypoint()
{
    PathFollowerParameters& opt = getOptions();
    Path::Ptr path = getPath();

    double tolerance = opt.wp_tolerance();

    if(controller_->getDirSign() < 0) {
        tolerance *= 2;
    }

    // if distance to wp < threshold
   // ROS_ERROR_STREAM_THROTTLE(2, "distance to wp: " << distanceTo(current_path[opt.wp_idx]) << " < " << tolerance);
    while(distanceTo(path->getCurrentWaypoint()) < tolerance) {
       // ROS_ERROR_STREAM("opt.wp_idx: " << opt.wp_idx << ", size: " << last_wp_idx);
        if(path->isLastWaypoint() || path->isSubPathDone()) {
            // if distance to wp == last_wp -> state = APPROACH_TURNING_POINT
            *status_ptr_ = path_msgs::FollowPathResult::RESULT_STATUS_MOVING;
            return new BehaviourApproachTurningPoint(parent_);
        }
        else {
            // else choose next wp
            path->switchToNextWaypoint();
        }
    }

    visualizer_->drawArrow(0, path->getCurrentWaypoint(), "current waypoint", 1, 1, 0);
    visualizer_->drawArrow(1, path->getLastWaypoint(), "current waypoint", 1, 0, 0);

    next_wp_map_.pose = path->getCurrentWaypoint();
    next_wp_map_.header.stamp = ros::Time::now();

    if ( !parent_.transformToLocal( next_wp_map_, next_wp_local_ )) {
        *status_ptr_ = path_msgs::FollowPathResult::RESULT_STATUS_TF_FAIL;
        throw EmergencyBreakException("cannot transform next waypoint",
                                      path_msgs::FollowPathResult::RESULT_STATUS_TF_FAIL);
    }

    return this;
}



//##### BEGIN BehaviourApproachTurningPoint

BehaviourApproachTurningPoint::BehaviourApproachTurningPoint(PathFollower &parent)
    : Behaviour(parent), done_(false)
{
    controller_->initApproachTurningPoint();
}

Behaviour* BehaviourApproachTurningPoint::execute(int *status)
{
    Behaviour* next = initExecute(status);
    if(next != this) {
        return next;
    }

    // check if point is reached
    //if(!done_) {
    controller_->setPath(getPath());
    done_ = controller_->behaveApproachTurningPoint();
    //}
    if (done_) {
        return handleDone();
    }

    return this;
}

Behaviour* BehaviourApproachTurningPoint::handleDone()
{
    controller_->stopMotion();

    if((std::abs(parent_.getVelocity().linear.x) > 0.01) ||
       (std::abs(parent_.getVelocity().linear.y) > 0.01) ||
       (std::abs(parent_.getVelocity().angular.z) > 0.01)) {
        ROS_INFO_THROTTLE(1, "WAITING until no more motion");
        *status_ptr_ = path_msgs::FollowPathResult::RESULT_STATUS_MOVING;

    } else {
        ROS_INFO("Done at waypoint -> reset");

        Path::Ptr path = getPath();
        path->switchToNextSubPath();

        if(path->isDone()) {
            *status_ptr_ = path_msgs::FollowPathResult::RESULT_STATUS_SUCCESS;
            return nullptr;
        } else {
            *status_ptr_ = path_msgs::FollowPathResult::RESULT_STATUS_MOVING;
            return new BehaviourOnLine(parent_);
        }
    }

    return this;
}

Behaviour* BehaviourApproachTurningPoint::selectNextWaypoint()
{
    Path::Ptr path = getPath();

    path->switchToLastWaypoint();

    visualizer_->drawArrow(0, path->getCurrentWaypoint(), "current waypoint", 1, 1, 0);
    visualizer_->drawArrow(1, path->getLastWaypoint(), "current waypoint", 1, 0, 0);

    next_wp_map_.pose = path->getCurrentWaypoint();
    next_wp_map_.header.stamp = ros::Time::now();

    if ( !parent_.transformToLocal( next_wp_map_, next_wp_local_ )) {
        *status_ptr_ = path_msgs::FollowPathResult::RESULT_STATUS_TF_FAIL;
        throw EmergencyBreakException("Cannot transform next waypoint",
                                      path_msgs::FollowPathResult::RESULT_STATUS_TF_FAIL);
    }

    return this;
}
