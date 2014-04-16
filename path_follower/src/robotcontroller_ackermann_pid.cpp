#include "robotcontroller_ackermann_pid.h"
#include "BehaviouralPathDriver.h"
#include "pathfollower.h"

#include <ros/ros.h>
#include <utils_general/MathHelper.h>

#include <sstream>

using namespace Eigen;
using namespace std;


namespace {
double sign(double value) {
    return (value > 0) - (value < 0);
}
}


RobotController_Ackermann_Pid::RobotController_Ackermann_Pid(BehaviouralPathDriver *path_driver):
    RobotController(path_driver)
{
}

void RobotController_Ackermann_Pid::configure()
{
    ros::NodeHandle nh("~");

    nh.param( "max_distance_to_path", opt_.max_distance_to_path_, 0.3 ); //TODO: find reasonable default value.

    double ta, kp, ki, i_max, delta_max, e_max;
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
//    BehaviouralPathDriver::Options& opt = getOptions();

//    double delta_f_raw = 0;

//    *status_ptr_ = path_msgs::FollowPathResult::MOTION_STATUS_MOVING;

//    if ( !getPid().execute( error, delta_f_raw)) {
//        // Nothing to do
//        return false;
//    }

//    drawSteeringArrow(14, parent_.getSlamPoseMsg(), delta_f_raw, 0.0, 1.0, 1.0);

//    double threshold = 5.0;
//    double threshold_max_distance = 3.5 /*m*/;

//    Path& current_path = getSubPath(opt.path_idx);
//    double distance_to_goal = current_path.back().distanceTo(current_path[opt.wp_idx]);
//    double threshold_distance = std::min(threshold_max_distance,
//                                         std::max((double) opt.collision_box_min_length_, distance_to_goal));

//    double delta_f;
//    VectorFieldHistogram& vfh = getVFH();
//    bool collision = false;

//    // FIXME: check if ~use_vfh == true
//    if(!vfh.isReady()) {
//        ROS_WARN_THROTTLE(1, "Not using VFH, not ready yet! (Maybe obstacle map not published?)");
//        delta_f = delta_f_raw;
//    } else {
//        vfh.create(threshold_distance, threshold);
//        collision = !vfh.adjust(delta_f_raw, threshold, delta_f);
//        vfh.visualize(delta_f_raw, threshold);
//    }

//    drawSteeringArrow(14, parent_.getSlamPoseMsg(), delta_f, 0.0, 1.0, 1.0);


//    BehaviouralPathDriver::Command& cmd = getCommand();


//    double steer = std::abs(delta_f);
//    ROS_DEBUG_STREAM("dir=" << dir_sign_ << ", steer=" << steer);
//    if(steer > getOptions().steer_slow_threshold_) {
//        ROS_WARN_STREAM_THROTTLE(2, "slowing down");
//        speed *= 0.5;
//    }

//    // make sure, the speed is in the allowed range
//    if (speed < getOptions().min_velocity_) {
//        speed = getOptions().min_velocity_;
//        ROS_WARN_THROTTLE(5, "Velocity is below minimum. It is set to minimum velocity.");
//    } else if (speed > getOptions().max_velocity_) {
//        speed = getOptions().max_velocity_;
//        ROS_WARN_THROTTLE(5, "Velocity is above maximum. Reduce to maximum velocity.");
//    }

//    cmd.steer_front = dir_sign_ * delta_f;
//    cmd.steer_back = 0;

//    collision |= isCollision(calculateCourse());

//    if(collision) {
//        ROS_WARN_THROTTLE(1, "Collision!");
//        *status_ptr_ = path_msgs::FollowPathResult::MOTION_STATUS_COLLISION; //FIXME: not so good to use result-constant if it is not finishing the action...

//        getCommand().velocity = 0;
//        parent_.publishCommand();

//    } else {
//        cmd.velocity = dir_sign_ * speed;
//    }

//    ROS_DEBUG("Set velocity to %g", speed);
//    return (std::abs(delta_f - delta_f_raw) > 0.05);
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

void RobotController_Ackermann_Pid::behaveOnLine(PathWithPosition path)
{
    //TODO: this whole initialization stuff should be packed in a method and the user should somehow be forced to call it.
    BehaviourDriveBase* behaviour = ((BehaviourDriveBase*) path_driver_->getActiveBehaviour());
    path_ = path;

    //TODO: not nice. can this transform also be done in path?
    geometry_msgs::PoseStamped wp_pose;
    wp_pose.header.stamp = ros::Time::now();
    wp_pose.pose = path.nextWaypoint();
    if ( !path_driver_->getNode()->transformToLocal( wp_pose, next_wp_local_)) {
        setStatus(path_msgs::FollowPathResult::MOTION_STATUS_SLAM_FAIL);
        throw new BehaviourEmergencyBreak(*path_driver_);
    }

    //---------------------


    dir_sign_ = sign(next_wp_local_.x());

    // Calculate target line from current to next waypoint (if there is any)
    double e_distance = calculateLineError();
    double e_angle = calculateAngleError();

    double e_combined = e_distance + e_angle;

    // draw steer front
    behaviour->drawSteeringArrow(1, path_driver_->getSlamPoseMsg(), e_angle, 0.2, 1.0, 0.2);
    behaviour->drawSteeringArrow(2, path_driver_->getSlamPoseMsg(), e_distance, 0.2, 0.2, 1.0);
    behaviour->drawSteeringArrow(3, path_driver_->getSlamPoseMsg(), e_combined, 1.0, 0.2, 0.2);

    float speed = velocity_;

    // TODO: better speed control
    //       - backwards not slower, but lower max speed
    //       - slower when close to goal, similar to ApproachGoal
    if(dir_sign_ < 0) {
        speed *= 0.5;
    }

    // if the robot is in this state, we assume that it is not avoiding any obstacles
    // so we abort, if robot moves too far from the path
    if (calculateDistanceToCurrentPathSegment() > opt_.max_distance_to_path_) {
        std::stringstream cmd;
        cmd << "espeak \"" << "abort: too far away!" << "\" 2> /dev/null 1> /dev/null &";
        system(cmd.str().c_str());

        ROS_WARN("Moved too far away from the path. Abort.");
        setStatus(path_msgs::FollowPathResult::MOTION_STATUS_PATH_LOST);
        throw new BehaviourEmergencyBreak(*path_driver_);
    }

    if(setCommand(e_combined, speed)) {
        setStatus(path_msgs::FollowPathResult::MOTION_STATUS_MOVING);
        throw new BehaviourAvoidObstacle(*path_driver_);
    }
}

void RobotController_Ackermann_Pid::behaveAvoidObstacle(PathWithPosition path)
{

}

void RobotController_Ackermann_Pid::behaveApproachTurningPoint(PathWithPosition path)
{

}

void RobotController_Ackermann_Pid::behaveEmergencyBreak()
{
    stopMotion();
}

void RobotController_Ackermann_Pid::setStatus(int status)
{
    ((BehaviourDriveBase*) path_driver_->getActiveBehaviour())->setStatus(status);
}

double RobotController_Ackermann_Pid::calculateAngleError()
{
    geometry_msgs::Pose waypoint = path_.nextWaypoint();
    geometry_msgs::Pose robot_pose = path_driver_->getSlamPoseMsg();
    return MathHelper::AngleClamp(tf::getYaw(waypoint.orientation) - tf::getYaw(robot_pose.orientation));
}

double RobotController_Ackermann_Pid::calculateLineError()
{
    BehaviourDriveBase* behaviour = ((BehaviourDriveBase*) path_driver_->getActiveBehaviour());

    geometry_msgs::PoseStamped followup_next_wp_map;
    followup_next_wp_map.header.stamp = ros::Time::now();

    if(path_.wp_idx + 1 == path_.current_path->size()) {
        followup_next_wp_map.pose = path_.getWaypoint(path_.wp_idx - 1);
    } else {
        followup_next_wp_map.pose = path_.getWaypoint(path_.wp_idx + 1);
    }

    Line2d target_line;
    Vector3d followup_next_wp_local;
    if (!path_driver_->getNode()->transformToLocal( followup_next_wp_map, followup_next_wp_local)) {
        setStatus(path_msgs::FollowPathResult::MOTION_STATUS_INTERNAL_ERROR);
        throw new BehaviourEmergencyBreak(*path_driver_);
    }
    target_line = Line2d( next_wp_local_.head<2>(), followup_next_wp_local.head<2>());
    behaviour->visualizeLine(target_line);

    Vector2d main_carrot, alt_carrot, front_pred, rear_pred;
    path_driver_->predictPose(front_pred, rear_pred);
    if(dir_sign_ >= 0) {
        main_carrot = front_pred;
        alt_carrot = rear_pred;
    } else {
        main_carrot = rear_pred;
        alt_carrot = front_pred;
    }

    behaviour->visualizeCarrot(main_carrot, 0, 1.0,0.0,0.0);
    behaviour->visualizeCarrot(alt_carrot, 1, 0.0,0.0,0.0);

    return -target_line.GetSignedDistance(main_carrot) - 0.25 * target_line.GetSignedDistance(alt_carrot);
}

double RobotController_Ackermann_Pid::calculateDistanceToCurrentPathSegment()
{
    /* Calculate line from last way point to current way point (which should be the line the robot is driving on)
     * and calculate the distance of the robot to this line.
     */


    assert(path_.wp_idx < (int) path_.current_path->size());

    // opt.wp_idx should be the index of the next waypoint. The last waypoint ist then simply wp_idx - 1.
    // Usually wp_idx is greater than zero, so this is possible.
    // There are, however, situations where wp_idx = 0. In this case the segment starting in wp_idx is used rather
    // than the one ending there (I am not absolutly sure if this a good behaviour, so observe this via debug-output).
    int wp1_idx = 0;
    if (path_.wp_idx > 0) {
        wp1_idx = path_.wp_idx - 1;
    } else {
        // if wp_idx == 0, use the segment from wp_idx to the following waypoint.
        wp1_idx = path_.wp_idx + 1;

        ROS_DEBUG("Toggle waypoints as wp_idx == 0 in calculateDistanceToCurrentPathSegment() (%s, line %d)", __FILE__, __LINE__);
    }

    geometry_msgs::Pose wp1 = path_.getWaypoint(wp1_idx);
    geometry_msgs::Pose wp2 = path_.getWaypoint(path_.wp_idx);

    // line from last waypoint to current one.
    Line2d segment_line(Vector2d(wp1.position.x, wp1.position.y), Vector2d(wp2.position.x, wp2.position.y));

    ///// visualize start and end point of the current segment (for debugging)
    Visualizer *vis = Visualizer::getInstance();
    vis->drawMark(24, wp1.position, "segment_marker", 0, 1, 1);
    vis->drawMark(25, wp2.position, "segment_marker", 1, 0, 1);
    /////

    // get distance of robot (slam_pose_) to segment_line.
    return segment_line.GetDistance(path_driver_->getSlamPose().head<2>());
}
