#include <path_follower/controller/robotcontroller.h>

/// PROJECT
#include <path_follower/pathfollower.h>
#include <utils_general/MathHelper.h>
#include <path_follower/utils/path_exceptions.h>

RobotController::RobotController(PathFollower* path_driver)
    : path_driver_(path_driver),
      velocity_(0.0f),
      dir_sign_(1.0f)
{
    initPublisher(&cmd_pub_);

    ros::NodeHandle& nh = path_driver->getNodeHandle();
    points_pub_ = nh.advertise<visualization_msgs::Marker>("path_points", 10);

    // path marker
    robot_path_marker_.header.frame_id = getFixedFrame();
    robot_path_marker_.header.stamp = ros::Time();
    robot_path_marker_.ns = "robot path";
    robot_path_marker_.id = 50;
    robot_path_marker_.type = visualization_msgs::Marker::LINE_STRIP;
    robot_path_marker_.action = visualization_msgs::Marker::ADD;
    robot_path_marker_.pose.position.x = 0;
    robot_path_marker_.pose.position.y = 0;
    robot_path_marker_.pose.position.z = 0;
    robot_path_marker_.pose.orientation.x = 0.0;
    robot_path_marker_.pose.orientation.y = 0.0;
    robot_path_marker_.pose.orientation.z = 0.0;
    robot_path_marker_.pose.orientation.w = 1.0;
    robot_path_marker_.scale.x = 0.01;
    robot_path_marker_.scale.y = 0.0;
    robot_path_marker_.scale.z = 0.0;
    robot_path_marker_.color.a = 1.0;
    robot_path_marker_.color.r = 0.0;
    robot_path_marker_.color.g = 0.0;
    robot_path_marker_.color.b = 1.0;

    visualizer_ = Visualizer::getInstance();
}

std::string RobotController::getFixedFrame() const
{
    if(path_) {
        return path_->getFrameId();
    } else {
        return "map";
    }
}

void RobotController::setStatus(int status)
{
    path_driver_->setStatus(status);
}

void RobotController::setPath(Path::Ptr path)
{
    path_ = path;

    robot_path_marker_.points.clear();

    geometry_msgs::PoseStamped wp_pose;
    wp_pose.header.stamp = ros::Time::now();
    wp_pose.pose = path->getCurrentWaypoint();
    if ( !path_driver_->transformToLocal( wp_pose, next_wp_local_)) {
        throw EmergencyBreakException("cannot transform path",
                                      path_msgs::FollowPathResult::RESULT_STATUS_TF_FAIL);
    }
}

void RobotController::setLocalPath(Path::Ptr path)
{
    // nothing to do, can be implemented by children
}

void RobotController::initPublisher(ros::Publisher *pub) const
{
    ros::NodeHandle nh;
    //TODO: implement a dynamic switching between velocity and torque mode
    //torque mode
    //*pub = nh.advertise<std_msgs::Float64MultiArray>("/wheel_torques", 10);
    //velocity mode
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
    case MoveCommandStatus::OKAY:
        return ControlStatus::OKAY;
    case MoveCommandStatus::REACHED_GOAL:
        return ControlStatus::REACHED_GOAL;
    default:
        ROS_ERROR("MoveCommandStatus %d is not handled by MCS2CS! Return ERROR instead.", s);
    case MoveCommandStatus::ERROR:
        return ControlStatus::ERROR;
    }
}

bool RobotController::isOmnidirectional() const
{
    return false;
}

void RobotController::publishPathMarker()
{
    Eigen::Vector3d current_pose = path_driver_->getRobotPose();
    geometry_msgs::Point pt;
    pt.x = current_pose[0];
    pt.y = current_pose[1];
    robot_path_marker_.points.push_back(pt);

    points_pub_.publish(robot_path_marker_);
}

RobotController::ControlStatus RobotController::execute()
{
    if(!path_) {
       return ControlStatus::OKAY;
    }
 
    publishPathMarker();

    MoveCommand cmd;
    MoveCommandStatus status = computeMoveCommand(&cmd);

    if (status != MoveCommandStatus::OKAY) {
        stopMotion();
        return MCS2CS(status);
    } else {
        bool cmd_modified = path_driver_->callObstacleAvoider(&cmd);

        if (!cmd.isValid()) {
            ROS_ERROR("Invalid move command.");
            stopMotion();
            return ControlStatus::ERROR;
        } else {
            publishMoveCommand(cmd);
            return cmd_modified ? ControlStatus::OBSTACLE : ControlStatus::OKAY;
        }
    }
}
