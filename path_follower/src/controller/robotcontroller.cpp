#include <path_follower/controller/robotcontroller.h>

/// PROJECT
#include <path_follower/pathfollower.h>
#include <utils_general/MathHelper.h>
#include <path_follower/utils/path_exceptions.h>


void RobotController::setStatus(int status)
{
    path_driver_->setStatus(status);
}

void RobotController::setPath(Path::Ptr path)
{
    path_ = path;

    geometry_msgs::PoseStamped wp_pose;
    wp_pose.header.stamp = ros::Time::now();
    wp_pose.pose = path->getCurrentWaypoint();
    if ( !path_driver_->transformToLocal( wp_pose, next_wp_local_)) {
        setStatus(path_msgs::FollowPathResult::MOTION_STATUS_SLAM_FAIL);
        throw EmergencyBreakException("cannot transform path");
    }
}

void RobotController::initPublisher(ros::Publisher *pub) const
{
    ros::NodeHandle nh; //TODO: does this work with only a local nh?
    *pub = nh.advertise<geometry_msgs::Twist> ("/cmd_vel", 10);
}

double RobotController::calculateAngleError()
{
    geometry_msgs::Pose waypoint   = path_->getCurrentWaypoint();
    geometry_msgs::Pose robot_pose = path_driver_->getRobotPoseMsg();
    return MathHelper::AngleClamp(tf::getYaw(waypoint.orientation) - tf::getYaw(robot_pose.orientation));
}

RobotController::ControlStatus RobotController::MCS2CS(RobotController::MoveCommandStatus s)
{
    switch (s) {
    case MC_OKAY:
        return OKAY;
    case MC_REACHED_GOAL:
        return REACHED_GOAL;
    default:
        ROS_ERROR("MoveCommandStatus %d is not handled by MCS2CS! Return ERROR instead.", s);
    case MC_ERROR:
        return ERROR;
    }
}

bool RobotController::isOmnidirectional() const
{
    return false;
}

RobotController::ControlStatus RobotController::execute()
{
    MoveCommand cmd;
    MoveCommandStatus status = computeMoveCommand(&cmd);

    /*TODO: Evlt wäre es sinnvoll den REACHED_GOAL check nicht in computeMoveCommand zu machen,
     * sondern in einer separaten Methode außerhalb, die einfach abstand zum ziel mit einer
     * bestimmten toleranz misst?
     */

    if (status != MC_OKAY) {
        return MCS2CS(status);
    } else {
        bool cmd_modified = path_driver_->callObstacleAvoider(&cmd);

        if (!cmd.isValid()) {
            ROS_ERROR("Invalid move command.");
            return ERROR;
        } else {
            publishMoveCommand(cmd);
            return cmd_modified ? OBSTACLE : OKAY;
        }
    }
}
