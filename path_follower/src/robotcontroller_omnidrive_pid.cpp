#include "robotcontroller_omnidrive_pid.h"

/// PROJECT
#include <utils_general/Line2d.h>
#include "BehaviouralPathDriver.h"
#include "pathfollower.h"
#include "behaviours.h"

using namespace Eigen;

RobotController_Omnidrive_Pid::RobotController_Omnidrive_Pid(ros::Publisher &cmd_publisher,
                                                             BehaviouralPathDriver *path_driver):
    RobotController(cmd_publisher, path_driver)
{
    visualizer_ = Visualizer::getInstance();

    configure();
}

void RobotController_Omnidrive_Pid::publishCommand()
{
    if (!cmd_.isValid()) {
        ROS_FATAL("Invalid Command in RobotController_Omnidrive_Pid.");
        return;
    }

    geometry_msgs::Twist msg = cmd_;
    cmd_pub_.publish(msg);

    setFilteredSpeed(cmd_.speed);
}

void RobotController_Omnidrive_Pid::stopMotion()
{
    cmd_.speed = 0;
    cmd_.direction_angle = 0;
    cmd_.rotation = 0;

    publishCommand();
}

void RobotController_Omnidrive_Pid::initOnLine()
{
    pid_direction_.reset();
    pid_rotation_.reset();
}

void RobotController_Omnidrive_Pid::behaveOnLine()
{
    // Calculate target line from current to next waypoint (if there is any)
    double e_distance = calculateLineError();
    double e_angle = calculateAngleError();

    ROS_DEBUG("OnLine: e_dist = %g, e_angle = %g", e_distance, e_angle);

    if (visualizer_->hasSubscriber()) {
        visualizer_->drawSteeringArrow(1, path_driver_->getSlamPoseMsg(), e_angle, 0.2, 1.0, 0.2);
        visualizer_->drawSteeringArrow(2, path_driver_->getSlamPoseMsg(), e_distance, 0.2, 0.2, 1.0);
    }

    //TODO: speed control!

    if(setCommand(e_distance, e_angle)) {
        // do not use this behaviour here, for the moment.
        //setStatus(path_msgs::FollowPathResult::MOTION_STATUS_MOVING);
        //throw new BehaviourAvoidObstacle(*path_driver_);
    }
}

void RobotController_Omnidrive_Pid::behaveAvoidObstacle()
{
    // not used
}

bool RobotController_Omnidrive_Pid::behaveApproachTurningPoint()
{
    //TODO
}

void RobotController_Omnidrive_Pid::configure()
{
    ros::NodeHandle nh("~");

    nh.param( "dead_time", options_.dead_time_, 0.10 );

    double ta, kp, ki, i_max, delta_max, e_max;
    nh.param( "pid/ta", ta, 0.03 );
    nh.param( "pid/kp", kp, 1.5 );
    nh.param( "pid/ki", ki, 0.001 );
    nh.param( "pid/i_max", i_max, 0.0 );
    nh.param( "pid/delta_max", delta_max, 30.0 );
    nh.param( "pid/e_max", e_max, 0.10 );

    //FIXME: separate parameters for the controlers...
    pid_direction_.configure( kp, ki, i_max, M_PI*delta_max/180.0, e_max, 0.5, ta );
    pid_rotation_.configure( kp, ki, i_max, M_PI*delta_max/180.0, e_max, 0.5, ta );
}

bool RobotController_Omnidrive_Pid::setCommand(double e_distance, double e_rotation)
{
    BehaviouralPathDriver::Options path_driver_opt = path_driver_->getOptions();
    BehaviourDriveBase* behaviour = ((BehaviourDriveBase*) path_driver_->getActiveBehaviour());

    float speed = velocity_;

    double delta_dir = 0, delta_rot = 0;

    setStatus(path_msgs::FollowPathResult::MOTION_STATUS_MOVING);

    if (!pid_direction_.execute(e_distance, delta_dir)
            && !pid_rotation_.execute(e_rotation, delta_rot)) {
        // Nothing to do
        return false;
    }
    ROS_DEBUG("PID-Direction: error = %g,\t delta = %g", e_distance, delta_dir);
    ROS_DEBUG("PID-Rotation:  error = %g,\t delta = %g", e_rotation, delta_rot);

    visualizer_->drawSteeringArrow(14, path_driver_->getSlamPoseMsg(), delta_dir, 0.0, 1.0, 1.0);


    bool collision = false;
    //TODO: VFH here


    double steer = std::abs(delta_dir);
    ROS_DEBUG_STREAM("dir=" << dir_sign_ << ", steer=" << steer);
    if(steer > path_driver_opt.steer_slow_threshold_) {
        ROS_WARN_STREAM_THROTTLE(2, "slowing down");
        speed *= 0.5;
    }

    // make sure, the speed is in the allowed range
    if (speed < path_driver_opt.min_velocity_) {
        speed = path_driver_opt.min_velocity_;
        ROS_WARN_THROTTLE(5, "Velocity is below minimum. It is set to minimum velocity.");
    } else if (speed > path_driver_opt.max_velocity_) {
        speed = path_driver_opt.max_velocity_;
        ROS_WARN_THROTTLE(5, "Velocity is above maximum. Reduce to maximum velocity.");
    }
    //TODO: bounds for rotational speed?


    cmd_.direction_angle = delta_dir;
    cmd_.rotation = delta_rot;


    collision |= behaviour->isCollision(cmd_.direction_angle);

    if(collision) {
        ROS_WARN_THROTTLE(1, "Collision!");
        setStatus(path_msgs::FollowPathResult::MOTION_STATUS_COLLISION); //FIXME: not so good to use result-constant if it is not finishing the action...

        stopMotion();
    } else {
        cmd_.speed = speed;
    }

    ROS_DEBUG("Set speed to %g", cmd_.speed);
    return false; // return true here, if VFH is at work?! (compare to ackermann_pid)
}

Eigen::Vector2d RobotController_Omnidrive_Pid::predictPosition()
{
    double dt = options_.dead_time_;
    double v  = getFilteredSpeed();
    double ds = dt*v;

    //FIXME ...
}

double RobotController_Omnidrive_Pid::calculateLineError()
{
    // control to the path segment after the current one -> determine next but one waypoint
    geometry_msgs::PoseStamped followup_next_wp_map;
    followup_next_wp_map.header.stamp = ros::Time::now();
    // use the current segment, if it is the last one
    if(path_.wp_idx + 1 == path_.current_path->size()) {
        followup_next_wp_map.pose = path_.getWaypoint(path_.wp_idx - 1);
    } else {
        followup_next_wp_map.pose = path_.getWaypoint(path_.wp_idx + 1);
    }

    // transform this waypoint to local frame
    Vector3d followup_next_wp_local;
    if (!path_driver_->getNode()->transformToLocal( followup_next_wp_map, followup_next_wp_local)) {
        setStatus(path_msgs::FollowPathResult::MOTION_STATUS_INTERNAL_ERROR);
        throw new BehaviourEmergencyBreak(*path_driver_);
    }

    // segment line
    Line2d target_line;
    target_line = Line2d( next_wp_local_.head<2>(), followup_next_wp_local.head<2>());
    visualizer_->visualizeLine(target_line);

    //FIXME: use predictPose()
    //Vector2d pred_pose = predictPose();
    //visualizer_->drawMark(0, );
    Vector3d pred_pose = path_driver_->getSlamPose();

    return -target_line.GetSignedDistance(pred_pose.head<2>());
}
