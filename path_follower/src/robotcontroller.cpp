#include "robotcontroller.h"

/// PROJECT
#include "BehaviouralPathDriver.h"
#include "pathfollower.h"
#include "behaviours.h"
#include <utils_general/MathHelper.h>


void RobotController::setStatus(int status)
{
    ((BehaviourDriveBase*) path_driver_->getActiveBehaviour())->setStatus(status);
}

void RobotController::setPath(PathWithPosition path)
{
    path_ = path;

    //TODO: not nice. can this transform also be done in path?
    geometry_msgs::PoseStamped wp_pose;
    wp_pose.header.stamp = ros::Time::now();
    wp_pose.pose = path.nextWaypoint();
    if ( !path_driver_->getNode()->transformToLocal( wp_pose, next_wp_local_)) {
        setStatus(path_msgs::FollowPathResult::MOTION_STATUS_SLAM_FAIL);
        throw new BehaviourEmergencyBreak(*path_driver_);
    }

    path_driver_->getNode()->path_lookout_.setPath(*path.current_path);
}

double RobotController::calculateAngleError()
{
    geometry_msgs::Pose waypoint   = path_.nextWaypoint();
    geometry_msgs::Pose robot_pose = path_driver_->getSlamPoseMsg();
    return MathHelper::AngleClamp(tf::getYaw(waypoint.orientation) - tf::getYaw(robot_pose.orientation));
}
