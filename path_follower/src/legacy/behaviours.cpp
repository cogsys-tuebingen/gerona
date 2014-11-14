#include <path_follower/legacy/behaviours.h>
#include <path_follower/pathfollower.h>
#include <utils_general/MathHelper.h>
#include <path_follower/utils/path_exceptions.h>

using namespace Eigen;

namespace {
double sign(double value) {
    if (value < 0) return -1;
    if (value > 0) return 1;
    return 0;
}
}


//##### BEGIN Behaviour
Behaviour::Behaviour(PathFollower &parent):
    parent_(parent),
    controller_(parent.getController())
{
    visualizer_ = Visualizer::getInstance();

    double wpto;
    ros::param::param<double>("~waypoint_timeout", wpto, 10.0); //TODO: wrap all these param accesses with Parameters class
    waypoint_timeout.duration = ros::Duration(wpto);
    waypoint_timeout.reset();
}

const SubPath& Behaviour::getCurrentSubPath()
{
    //return parent_.paths_[index];
    return parent_.paths_->getCurrentSubPath();
}
size_t Behaviour::getSubPathCount() const
{
    return parent_.paths_->subPathCount();
}

double Behaviour::distanceTo(const Waypoint& wp)
{
    Eigen::Vector3d pose = parent_.getRobotPose();
    return hypot(pose(0) - wp.x, pose(1) - wp.y);
}

double Behaviour::calculateDistanceToCurrentPathSegment()
{
    /* Calculate line from last way point to current way point (which should be the line the robot is driving on)
     * and calculate the distance of the robot to this line.
     */

    Path::Ptr path = getPath();

    // Get previous waypoint
    // If the current waypoint is the first in this sub path (i.e. index == 0), use the next
    // instead. (I am not absolutly sure if this a good behaviour, so observe this via debug-output).
    int wp1_idx = 0;
    if (path->getWaypointIndex() > 0) {
        wp1_idx = path->getWaypointIndex() - 1;
    } else {
        // if wp_idx == 0, use the segment from 0th to 1st waypoint.
        wp1_idx = 1;

        ROS_DEBUG("Toggle waypoints as wp_idx == 0 in calculateDistanceToCurrentPathSegment() (%s, line %d)", __FILE__, __LINE__);
    }

    geometry_msgs::Pose wp1 = path->getWaypoint(wp1_idx);
    geometry_msgs::Pose wp2 = path->getCurrentWaypoint();

    // line from last waypoint to current one.
    Line2d segment_line(Vector2d(wp1.position.x, wp1.position.y), Vector2d(wp2.position.x, wp2.position.y));

    ///// visualize start and end point of the current segment (for debugging)
    visualizer_->drawMark(24, wp1.position, "segment_marker", 0, 1, 1);
    visualizer_->drawMark(25, wp2.position, "segment_marker", 1, 0, 1);
    /////

    // get distance of robot (slam_pose_) to segment_line.
    return segment_line.GetDistance(parent_.getRobotPose().head<2>());
}

void Behaviour::setStatus(int status)
{
    *status_ptr_ = status;
}


Behaviour* Behaviour::initExecute(int *status)
{
    status_ptr_ = status;

    Behaviour* next_behaviour = selectNextWaypoint();
    if(next_behaviour != this) {
        return next_behaviour;
    }

    if (waypoint_timeout.isExpired()) {
        ROS_WARN("Waypoint Timeout! The robot did not reach the next waypoint within %g sec. Abort path execution.",
                 waypoint_timeout.duration.toSec());
        *status_ptr_ = path_msgs::FollowPathResult::MOTION_STATUS_TIMEOUT;
        return new BehaviourEmergencyBreak(parent_);
    }

    if (!isLeavingPathAllowed()) {
        double dist = calculateDistanceToCurrentPathSegment();
        //ROS_DEBUG("Distance to current path segment: %g m", dist);
        if (dist > getOptions().max_distance_to_path()) {
            parent_.say("abort: too far away!");

            ROS_WARN("Moved too far away from the path (%g m, limit: %g m). Abort.",
                     calculateDistanceToCurrentPathSegment(),
                     getOptions().max_distance_to_path());

            setStatus(path_msgs::FollowPathResult::MOTION_STATUS_PATH_LOST);
            return new BehaviourEmergencyBreak(parent_);
        }
    }

    return this;
}

Path::Ptr Behaviour::getPath()
{
    return parent_.getPath();
}

VectorFieldHistogram& Behaviour::getVFH()
{
    return parent_.getVFH();
}
PathFollower::Options& Behaviour::getOptions()
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
    PathFollower::Options& opt = getOptions();
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
            *status_ptr_ = path_msgs::FollowPathResult::MOTION_STATUS_MOVING;
            return new BehaviourApproachTurningPoint(parent_);
        }
        else {
            // else choose next wp
            path->switchToNextWaypoint();

            waypoint_timeout.reset();
        }
    }

    visualizer_->drawArrow(0, path->getCurrentWaypoint(), "current waypoint", 1, 1, 0);
    visualizer_->drawArrow(1, path->getLastWaypoint(), "current waypoint", 1, 0, 0);

    next_wp_map_.pose = path->getCurrentWaypoint();
    next_wp_map_.header.stamp = ros::Time::now();

    if ( !parent_.transformToLocal( next_wp_map_, next_wp_local_ )) {
        *status_ptr_ = path_msgs::FollowPathResult::MOTION_STATUS_SLAM_FAIL;
        throw new EmergencyBreakException("cannot transform next waypoint");
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
        *status_ptr_ = path_msgs::FollowPathResult::MOTION_STATUS_MOVING;

    } else {
        ROS_INFO("Done at waypoint -> reset");

        Path::Ptr path = getPath();
        path->switchToNextSubPath();

        if(path->isDone()) {
            *status_ptr_ = path_msgs::FollowPathResult::MOTION_STATUS_SUCCESS;
            return NULL;
        } else {
            *status_ptr_ = path_msgs::FollowPathResult::MOTION_STATUS_MOVING;
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
        *status_ptr_ = path_msgs::FollowPathResult::MOTION_STATUS_SLAM_FAIL;
        throw new EmergencyBreakException("Cannot transform next waypoint");
    }

    return this;
}
