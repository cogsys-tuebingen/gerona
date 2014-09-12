#include "behaviours.h"
#include "pathfollower.h"
#include <utils_general/MathHelper.h>

using namespace Eigen;

namespace {
double sign(double value) {
    if (value < 0) return -1;
    if (value > 0) return 1;
    return 0;
}
}


//##### BEGIN Behaviour
Path& Behaviour::getSubPath(unsigned index)
{
    return parent_.paths_[index];
}
int Behaviour::getSubPathCount() const
{
    return parent_.paths_.size();
}

double Behaviour::distanceTo(const Waypoint& wp)
{
    Eigen::Vector3d pose = parent_.getRobotPose();
    return hypot(pose(0) - wp.x, pose(1) - wp.y);
}
PathFollower::Command& Behaviour::getCommand()
{
    return parent_.current_command_;
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


//##### BEGIN BehaviourDriveBase
BehaviourDriveBase::BehaviourDriveBase(PathFollower &parent)
    : Behaviour(parent)
{
    visualizer_ = Visualizer::getInstance();

    double wpto;
    ros::param::param<double>("~waypoint_timeout", wpto, 10.0);
    waypoint_timeout.duration = ros::Duration(wpto);
    waypoint_timeout.reset();
}

double BehaviourDriveBase::calculateDistanceToCurrentPathSegment()
{
    /* Calculate line from last way point to current way point (which should be the line the robot is driving on)
     * and calculate the distance of the robot to this line.
     */

    PathFollower::Options& opt = getOptions();
    Path& current_path = getSubPath(opt.path_idx);

    assert(opt.wp_idx < (int) current_path.size());

    // opt.wp_idx should be the index of the next waypoint. The last waypoint ist then simply wp_idx - 1.
    // Usually wp_idx is greater than zero, so this is possible.
    // There are, however, situations where wp_idx = 0. In this case the segment starting in wp_idx is used rather
    // than the one ending there (I am not absolutly sure if this a good behaviour, so observe this via debug-output).
    int wp1_idx = 0;
    if (opt.wp_idx > 0) {
        wp1_idx = opt.wp_idx - 1;
    } else {
        // if wp_idx == 0, use the segment from wp_idx to the following waypoint.
        wp1_idx = opt.wp_idx + 1;

        ROS_DEBUG("Toggle waypoints as wp_idx == 0 in calculateDistanceToCurrentPathSegment() (%s, line %d)", __FILE__, __LINE__);
    }

    geometry_msgs::Pose wp1 = current_path[wp1_idx];
    geometry_msgs::Pose wp2 = current_path[opt.wp_idx];

    // line from last waypoint to current one.
    Line2d segment_line(Vector2d(wp1.position.x, wp1.position.y), Vector2d(wp2.position.x, wp2.position.y));

    ///// visualize start and end point of the current segment (for debugging)
    visualizer_->drawMark(24, wp1.position, "segment_marker", 0, 1, 1);
    visualizer_->drawMark(25, wp2.position, "segment_marker", 1, 0, 1);
    /////

    // get distance of robot (slam_pose_) to segment_line.
    return segment_line.GetDistance(parent_.getRobotPose().head<2>());
}

void BehaviourDriveBase::setStatus(int status)
{
    *status_ptr_ = status;
}

void BehaviourDriveBase::initExecute(int *status)
{
    status_ptr_ = status;

    getNextWaypoint();
    checkWaypointTimeout();
    checkDistanceToPath();
}

void BehaviourDriveBase::checkWaypointTimeout()
{
    if (waypoint_timeout.isExpired()) {
        ROS_WARN("Waypoint Timeout! The robot did not reach the next waypoint within %g sec. Abort path execution.",
                 waypoint_timeout.duration.toSec());
        *status_ptr_ = path_msgs::FollowPathResult::MOTION_STATUS_TIMEOUT;
        throw new BehaviourEmergencyBreak(parent_);
    }
}

void BehaviourDriveBase::checkDistanceToPath()
{
    if (!isLeavingPathAllowed()) {
        double dist = calculateDistanceToCurrentPathSegment();
        ROS_DEBUG("Distance to current path segment: %g m", dist);
        if (dist > getOptions().max_distance_to_path_) {
            parent_.say("abort: too far away!");

            ROS_WARN("Moved too far away from the path (%g m, limit: %g m). Abort.",
                     calculateDistanceToCurrentPathSegment(),
                     getOptions().max_distance_to_path_);

            setStatus(path_msgs::FollowPathResult::MOTION_STATUS_PATH_LOST);
            throw new BehaviourEmergencyBreak(parent_);
        }
    }
}

PathWithPosition BehaviourDriveBase::getPathWithPosition()
{
    PathFollower::Options& opt = getOptions();
    Path& current_path = getSubPath(opt.path_idx);
    return PathWithPosition(&current_path, opt.wp_idx);
}




//##### BEGIN BehaviourOnLine

BehaviourOnLine::BehaviourOnLine(PathFollower& parent)
    : BehaviourDriveBase(parent)
{
    controller_->initOnLine();
}


void BehaviourOnLine::execute(int *status)
{
    initExecute(status);

    controller_->execBehaviourOnLine(getPathWithPosition());
}


void BehaviourOnLine::getNextWaypoint()
{
    PathFollower::Options& opt = getOptions();
    Path& current_path = getSubPath(opt.path_idx);

    assert(opt.wp_idx < (int) current_path.size());

    int last_wp_idx = current_path.size() - 1;

    double tolerance = opt.wp_tolerance_;

    if(controller_->getDirSign() < 0) {
        tolerance *= 2;
    }

    // if distance to wp < threshold
    ROS_ERROR_STREAM_THROTTLE(2, "distance to wp: " << distanceTo(current_path[opt.wp_idx]) << " < " << tolerance);
    while(distanceTo(current_path[opt.wp_idx]) < tolerance) {
        ROS_ERROR_STREAM("opt.wp_idx: " << opt.wp_idx << ", size: " << last_wp_idx);
        if(opt.wp_idx >= last_wp_idx) {
            // if distance to wp == last_wp -> state = APPROACH_TURNING_POINT
            *status_ptr_ = path_msgs::FollowPathResult::MOTION_STATUS_MOVING;
            throw new BehaviourApproachTurningPoint(parent_);
        }
        else {
            // else choose next wp
            opt.wp_idx++;

            waypoint_timeout.reset();
        }
    }

    visualizer_->drawArrow(0, current_path[opt.wp_idx], "current waypoint", 1, 1, 0);
    visualizer_->drawArrow(1, current_path[last_wp_idx], "current waypoint", 1, 0, 0);

    next_wp_map_.pose = current_path[opt.wp_idx];
    next_wp_map_.header.stamp = ros::Time::now();

    if ( !parent_.transformToLocal( next_wp_map_, next_wp_local_ )) {
        *status_ptr_ = path_msgs::FollowPathResult::MOTION_STATUS_SLAM_FAIL;
        throw new BehaviourEmergencyBreak(parent_);
    }
}



//##### BEGIN BehaviourAvoidObstacle
void BehaviourAvoidObstacle::execute(int *status)
{
    initExecute(status);

    controller_->execBehaviourAvoidObstacle(getPathWithPosition());
}



void BehaviourAvoidObstacle::getNextWaypoint()
{
    // TODO: improve!
    PathFollower::Options& opt = getOptions();
    Path& current_path = getSubPath(opt.path_idx);

    assert(opt.wp_idx < (int) current_path.size());

    int last_wp_idx = current_path.size() - 1;

    double tolerance = opt.wp_tolerance_;

    if(controller_->getDirSign() < 0) {
        tolerance *= 2;
    }

    // if distance to wp < threshold
    while(distanceTo(current_path[opt.wp_idx]) < tolerance) {
        if(opt.wp_idx >= last_wp_idx) {
            // if distance to wp == last_wp -> state = APPROACH_TURNING_POINT
            *status_ptr_ = path_msgs::FollowPathResult::MOTION_STATUS_MOVING;
            throw new BehaviourApproachTurningPoint(parent_);
        }
        else {
            // else choose next wp
            opt.wp_idx++;

            waypoint_timeout.reset();
        }
    }

    visualizer_->drawArrow(0, current_path[opt.wp_idx], "current waypoint", 1, 1, 0);
    visualizer_->drawArrow(1, current_path[last_wp_idx], "current waypoint", 1, 0, 0);

    next_wp_map_.pose = current_path[opt.wp_idx];
    next_wp_map_.header.stamp = ros::Time::now();

    if ( !parent_.transformToLocal( next_wp_map_, next_wp_local_ )) {
        *status_ptr_ = path_msgs::FollowPathResult::MOTION_STATUS_SLAM_FAIL;
        throw new BehaviourEmergencyBreak(parent_);
    }
}


//##### BEGIN BehaviourApproachTurningPoint

BehaviourApproachTurningPoint::BehaviourApproachTurningPoint(PathFollower &parent)
    : BehaviourDriveBase(parent), done_(false)
{
    controller_->initApproachTurningPoint();
}

void BehaviourApproachTurningPoint::execute(int *status)
{
    initExecute(status);

    // check if point is reached
    //if(!done_) {
    done_ = controller_->execBehaviourApproachTurningPoint(getPathWithPosition());
    //}
    if (done_) {
        handleDone();
    }
}

void BehaviourApproachTurningPoint::handleDone()
{
    controller_->stopMotion();

    if((std::abs(parent_.getVelocity().linear.x) > 0.01) ||
       (std::abs(parent_.getVelocity().linear.y) > 0.01) ||
       (std::abs(parent_.getVelocity().angular.z) > 0.01)) {
        ROS_INFO_THROTTLE(1, "WAITING until no more motion");
        *status_ptr_ = path_msgs::FollowPathResult::MOTION_STATUS_MOVING;
    } else {
        ROS_INFO("Done at waypoint -> reset");

        PathFollower::Options& opt = getOptions();

        opt.path_idx++;
        opt.wp_idx = 0;

        controller_->reset();

        if(opt.path_idx < getSubPathCount()) {
            *status_ptr_ = path_msgs::FollowPathResult::MOTION_STATUS_MOVING;
            throw new BehaviourOnLine(parent_);

        } else {
            *status_ptr_ = path_msgs::FollowPathResult::MOTION_STATUS_SUCCESS;
            throw new NullBehaviour;
        }
    }
}

void BehaviourApproachTurningPoint::getNextWaypoint()
{
    PathFollower::Options& opt = getOptions();
    Path& current_path = getSubPath(opt.path_idx);

    assert(opt.wp_idx < (int) current_path.size());

    int last_wp_idx = current_path.size() - 1;
    opt.wp_idx = last_wp_idx;

    visualizer_->drawArrow(0, current_path[opt.wp_idx], "current waypoint", 1, 1, 0);
    visualizer_->drawArrow(1, current_path[last_wp_idx], "current waypoint", 1, 0, 0);

    next_wp_map_.pose = current_path[opt.wp_idx];
    next_wp_map_.header.stamp = ros::Time::now();

    if ( !parent_.transformToLocal( next_wp_map_, next_wp_local_ )) {
        *status_ptr_ = path_msgs::FollowPathResult::MOTION_STATUS_SLAM_FAIL;
        throw new BehaviourEmergencyBreak(parent_);
    }
}
