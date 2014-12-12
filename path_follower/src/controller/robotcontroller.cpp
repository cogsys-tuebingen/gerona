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

bool RobotController::isOmnidirectional() const
{
    return false;
}

RobotController::ControlStatus RobotController::execute()
{
    MoveCommand cmd;
    ControlStatus status = computeMoveCommand(&cmd);

    /*TODO: das ganze konzept mit dem ControlStatus passt hier nicht mehr so richtig.
     * computeMoveCommand hat keine Ahnung von Hindernissen, sondern kann nur sagen, ob
     * die Berechnung des MoveCommand erfolgreich war oder nicht.
     * Evlt wäre es auch sinnvoll den SUCCESS check nicht dort, sondern in einer separaten
     * Methode zu machen?
     *
     *TODO: Status 'MOVING' should be renamed as depending on the obstacle avoider, the robot
     * might also move when status is OBSTACLE.
     */

    if (status != MOVING) {
        return status;
    } else {
        bool cmd_modified = path_driver_->callObstacleAvoider(&cmd);

        if (!cmd.isValid()) {
            ROS_ERROR("Invalid move command.");
            return ERROR;
        } else {
            publishMoveCommand(cmd);
            return cmd_modified ? OBSTACLE : MOVING;
        }
    }
}
