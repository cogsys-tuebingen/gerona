#include <path_follower/controller/robotcontroller.h>

/// PROJECT
#include <path_follower/pathfollower.h>
#include <utils_general/MathHelper.h>
#include <path_follower/utils/path_exceptions.h>


void RobotController::setStatus(int status)
{
    path_driver_->setStatus(status);
}

void RobotController::setPath(PathWithPosition path)
{
    path_ = path;

    geometry_msgs::PoseStamped wp_pose;
    wp_pose.header.stamp = ros::Time::now();
    wp_pose.pose = path.nextWaypoint();
    if ( !path_driver_->transformToLocal( wp_pose, next_wp_local_)) {
        setStatus(path_msgs::FollowPathResult::MOTION_STATUS_SLAM_FAIL);
        throw EmergencyBreakException("cannot transform path");
    }

    if (path_driver_->getOptions().use_path_lookout())
        path_driver_->getPathLookout()->setPath(path);
}

double RobotController::calculateAngleError()
{
    geometry_msgs::Pose waypoint   = path_.nextWaypoint();
    geometry_msgs::Pose robot_pose = path_driver_->getRobotPoseMsg();
    return MathHelper::AngleClamp(tf::getYaw(waypoint.orientation) - tf::getYaw(robot_pose.orientation));
}

bool RobotController::isOmnidirectional() const
{
    return false;
}
