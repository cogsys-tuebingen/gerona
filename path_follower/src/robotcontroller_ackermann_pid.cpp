#include "robotcontroller_ackermann_pid.h"
#include "BehaviouralPathDriver.h"

#include <ros/ros.h>
#include <utils_general/MathHelper.h>

RobotController_Ackermann_Pid::RobotController_Ackermann_Pid(BehaviouralPathDriver *path_driver):
    RobotController(path_driver)
{
}

void RobotController_Ackermann_Pid::configure()
{
    ros::NodeHandle nh("~");    double ta, kp, ki, i_max, delta_max, e_max;

    nh.param( "pid/ta", ta, 0.03 );
    nh.param( "pid/kp", kp, 1.5 );
    nh.param( "pid/ki", ki, 0.001 );
    nh.param( "pid/i_max", i_max, 0.0 );
    nh.param( "pid/delta_max", delta_max, 30.0 );
    nh.param( "pid/e_max", e_max, 0.10 );

    pid_.configure( kp, ki, i_max, M_PI*delta_max/180.0, e_max, 0.5, ta );
}

bool RobotController_Ackermann_Pid::setCommand(double error, double speed)
{

}

void RobotController_Ackermann_Pid::publishCommand()
{
//    //ramaxx_msgs::RamaxxMsg msg = current_command_;
//    geometry_msgs::Twist msg = current_command_;
//    cmd_pub_.publish(msg);

//    setFilteredSpeed(current_command_.velocity);
}

void RobotController_Ackermann_Pid::stopMotion()
{
    cmd_.velocity = 0;
    cmd_.steer_front = 0;
    cmd_.steer_back= 0;

    publishCommand();
}

void RobotController_Ackermann_Pid::behaveOnLine() // hier als parameter current_path und wp_id uebergeben?
{
//    dir_sign_ = sign(next_wp_local_.x());

//    // Calculate target line from current to next waypoint (if there is any)
//    double e_distance = calculateLineError();
//    double e_angle = calculateAngleError();

//    double e_combined = e_distance + e_angle;

//    // draw steer front
//    drawSteeringArrow(1, slam_pose_msg_, e_angle, 0.2, 1.0, 0.2);
//    drawSteeringArrow(2, slam_pose_msg_, e_distance, 0.2, 0.2, 1.0);
//    drawSteeringArrow(3, slam_pose_msg_, e_combined, 1.0, 0.2, 0.2);

//    float speed = getOptions().velocity_;

//    // TODO: better speed control
//    //       - backwards not slower, but lower max speed
//    //       - slower when close to goal, similar to ApproachGoal
//    if(dir_sign_ < 0) {
//        speed *= 0.5;
//    }

//    // if the robot is in this state, we assume that it is not avoiding any obstacles
//    // so we abort, if robot moves too far from the path
//    if (calculateDistanceToCurrentPathSegment() > getOptions().max_distance_to_path_) {

//    std::stringstream cmd;
//    cmd << "espeak \"" << "abort: too far away!" << "\" 2> /dev/null 1> /dev/null &";
//    system(cmd.str().c_str());

//        ROS_WARN("Moved too far away from the path. Abort.");
//        *status_ptr_ = path_msgs::FollowPathResult::MOTION_STATUS_PATH_LOST;
//        throw new BehaviourEmergencyBreak(parent_);
//    }

//    if(setCommand(e_combined, speed)) {
//        *status_ptr_ = path_msgs::FollowPathResult::MOTION_STATUS_MOVING;
//        throw new BehaviourAvoidObstacle(parent_);
    //    }
}

void RobotController_Ackermann_Pid::behaveAvoidObstacle()
{

}

void RobotController_Ackermann_Pid::behaveApproachTurningPoint()
{

}

void RobotController_Ackermann_Pid::behaveEmergencyBreak()
{
    stopMotion();
}

double RobotController_Ackermann_Pid::calculateAngleError()
{
    geometry_msgs::Pose robot_pose = path_driver_->getSlamPoseMsg();
//    return MathHelper::AngleClamp(tf::getYaw(next_wp_map_.pose.orientation) - tf::getYaw(robot_pose.orientation));
}

double RobotController_Ackermann_Pid::calculateLineError()
{
//    BehaviouralPathDriver::Options& opt = getOptions();
//    BehaviouralPathDriver::Path& current_path = getSubPath(opt.path_idx);

//    geometry_msgs::PoseStamped followup_next_wp_map;
//    followup_next_wp_map.header.stamp = ros::Time::now();

//    if(opt.wp_idx + 1 == current_path.size()) {
//        followup_next_wp_map.pose = current_path[opt.wp_idx - 1];
//    } else {
//        followup_next_wp_map.pose = current_path[opt.wp_idx + 1];
//    }

//    Line2d target_line;
//    Vector3d followup_next_wp_local;
//    if (!getNode().transformToLocal( followup_next_wp_map, followup_next_wp_local)) {
//        *status_ptr_ = path_msgs::FollowPathResult::MOTION_STATUS_INTERNAL_ERROR;
//        throw new BehaviourEmergencyBreak(parent_);
//    }
//    target_line = Line2d( next_wp_local_.head<2>(), followup_next_wp_local.head<2>());
//    visualizeLine(target_line);

//    Vector2d main_carrot, alt_carrot, front_pred, rear_pred;
//    parent_.predictPose(front_pred, rear_pred);
//    if(dir_sign_ >= 0) {
//        main_carrot = front_pred;
//        alt_carrot = rear_pred;
//    } else {
//        main_carrot = rear_pred;
//        alt_carrot = front_pred;
//    }

//    visualizeCarrot(main_carrot, 0, 1.0,0.0,0.0);
//    visualizeCarrot(alt_carrot, 1, 0.0,0.0,0.0);

//    return -target_line.GetSignedDistance(main_carrot) - 0.25 * target_line.GetSignedDistance(alt_carrot);
}
