#include "robotcontroller_omnidrive_pid.h"

RobotController_Omnidrive_Pid::RobotController_Omnidrive_Pid(ros::Publisher &cmd_publisher,
                                                             BehaviouralPathDriver *path_driver):
    RobotController(cmd_publisher, path_driver)
{
    configure();
}

void RobotController_Omnidrive_Pid::initOnLine()
{
    pid_.reset();
}

void RobotController_Omnidrive_Pid::behaveOnLine(PathWithPosition path)
{
    setPath(path);
    //---------------------


//    // Calculate target line from current to next waypoint (if there is any)
//    double e_distance = calculateLineError();
//    double e_angle = calculateAngleError();

//    double e_combined = e_distance + e_angle;
//    ROS_DEBUG("OnLine: e_dist = %g, e_angle = %g  ==>  e_comb = %g", e_distance, e_angle, e_combined);

//    // draw steer front
//    if (visualizer_->hasSubscriber()) {
//        visualizer_->drawSteeringArrow(1, path_driver_->getSlamPoseMsg(), e_angle, 0.2, 1.0, 0.2);
//        visualizer_->drawSteeringArrow(2, path_driver_->getSlamPoseMsg(), e_distance, 0.2, 0.2, 1.0);
//        visualizer_->drawSteeringArrow(3, path_driver_->getSlamPoseMsg(), e_combined, 1.0, 0.2, 0.2);
//    }

//    float speed = velocity_;

//    // TODO: better speed control
//    //       - backwards not slower, but lower max speed
//    //       - slower when close to goal, similar to ApproachGoal
//    if(dir_sign_ < 0) {
//        speed *= 0.5;
//    }

//    if(setCommand(e_combined, speed)) {
//        setStatus(path_msgs::FollowPathResult::MOTION_STATUS_MOVING);
//        throw new BehaviourAvoidObstacle(*path_driver_);
//    }
}

void RobotController_Omnidrive_Pid::configure()
{
    ros::NodeHandle nh("~");

    double ta, kp, ki, i_max, delta_max, e_max;
    nh.param( "pid/ta", ta, 0.03 );
    nh.param( "pid/kp", kp, 1.5 );
    nh.param( "pid/ki", ki, 0.001 );
    nh.param( "pid/i_max", i_max, 0.0 );
    nh.param( "pid/delta_max", delta_max, 30.0 );
    nh.param( "pid/e_max", e_max, 0.10 );

    pid_.configure( kp, ki, i_max, M_PI*delta_max/180.0, e_max, 0.5, ta );
}
