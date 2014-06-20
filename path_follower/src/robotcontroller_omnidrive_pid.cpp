#include "robotcontroller_omnidrive_pid.h"

/// PROJECT
#include <utils_general/Line2d.h>
#include <utils_general/MathHelper.h>
#include "BehaviouralPathDriver.h"
#include "pathfollower.h"
#include "behaviours.h"

using namespace Eigen;

RobotController_Omnidrive_Pid::RobotController_Omnidrive_Pid(ros::Publisher &cmd_publisher,
                                                             BehaviouralPathDriver *path_driver):
    RobotController(cmd_publisher, path_driver),
    pids_(2),
    cmd_(this),
    last_position_direction_update_(0,0),
    smoothed_direction_(0,0)
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
    pids_.resetAll();
    has_last_position_ = false;
    has_last_position_smoothed_ = false;
}

void RobotController_Omnidrive_Pid::behaveOnLine()
{
    // Calculate target line from current to next waypoint (if there is any)
    double e_direction = calculateDirectionError();
    double e_angle     = calculateAngleError();

    ROS_DEBUG("OnLine: e_dir = %g, e_angle = %g", e_direction, e_angle);

    if (visualizer_->hasSubscriber()) {
        visualizer_->drawSteeringArrow(1, path_driver_->getSlamPoseMsg(), e_angle, 0.2, 1.0, 0.2);
        visualizer_->drawSteeringArrow(2, path_driver_->getSlamPoseMsg(), e_direction, 0.2, 0.2, 1.0);
    }

    //TODO: speed control!

    if(setCommand(e_direction, e_angle, velocity_)) {
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
    // Check if done
    if (checkIfTurningPointApproached()) {
        return true;
    }

    // Calculate target line from current to next waypoint (if there is any)
    //double e_distance = calculateDistanceToWaypoint();
    double e_direction = calculateDirectionError();
    double e_angle     = calculateAngleError();

    ROS_DEBUG("Approach: e_dir = %g, e_angle = %g", e_direction, e_angle);

    if (visualizer_->hasSubscriber()) {
        visualizer_->drawCircle(2, ((geometry_msgs::Pose) path_.nextWaypoint()).position, 0.5, "/map", "turning point", 1, 1, 1);
        visualizer_->drawSteeringArrow(1, path_driver_->getSlamPoseMsg(), e_angle, 0.2, 1.0, 0.2);
        visualizer_->drawSteeringArrow(2, path_driver_->getSlamPoseMsg(), e_direction, 0.2, 0.2, 1.0);
    }

    float distance = std::sqrt(next_wp_local_.dot(next_wp_local_));
    float velocity = std::max(0.1f + distance / 2.0f, path_driver_->getOptions().min_velocity_);

    setCommand(e_direction, e_angle, velocity);

    return false;
}

void RobotController_Omnidrive_Pid::configure()
{
    ros::NodeHandle nh("~");

    nh.param( "dead_time", options_.dead_time_, 0.10 );

    double ta, kp, ki, i_max, delta_max=0, e_max=0;
    nh.param( "pid/ta", ta, 0.03 );
    pids_.setTa(ta);

    nh.param( "pid/direction/kp", kp, 0.1 );
    nh.param( "pid/direction/ki", ki, 0.0 );
    nh.param( "pid/direction/i_max", i_max, 0.0 );
//    nh.param( "pid/delta_max", delta_max, 30.0 ); //not used
//    nh.param( "pid/e_max", e_max, 0.10 );
    pids_.configure(DIRECTION, kp, M_PI*delta_max/180.0, e_max, 0.5, ki, i_max);

    nh.param( "pid/orientation/kp", kp, 1.0 );
    nh.param( "pid/orientation/ki", ki, 0.0 );
    nh.param( "pid/orientation/i_max", i_max, 0.0 );
    pids_.configure(ORIENTATION, kp, M_PI*delta_max/180.0, e_max, 0.5, ki, i_max);
}

bool RobotController_Omnidrive_Pid::checkIfTurningPointApproached() const
{
    Waypoint next_wp = path_.nextWaypoint();


    /*** first check if the orientation is ok ***/

    double orientation_diff = MathHelper::AngleClamp( next_wp.orientation - path_driver_->getSlamPose()[2]);
    bool reached_orientation = abs(orientation_diff) < 10 * M_PI/180; // 10° tolerance. - parameter for this?

    // if not done here, there is no need to check the position.
    if (!reached_orientation) {
        return false;
    }

    /******************************************************************************************
     * If this line is reached, the robot's orientation fits the one of the waypoint.
     * Now the more complicated check of the position is still to be done (this behaviour is
     * only finished, if the robot matches the in orientation AND position with the waypoint).
     *****************************************************************************************/

    //! Difference of current robot pose to the next waypoint.
    Vector2d delta;
    delta << next_wp.x - path_driver_->getSlamPoseMsg().position.x,
             next_wp.y - path_driver_->getSlamPoseMsg().position.y;

    //! Unit vector pointing in the direction of the next waypoints orientation.
    Vector2d target_dir;
    target_dir << std::cos(next_wp.orientation), std::sin(next_wp.orientation);

    // atan2(y,x) = angle of the vector.
    //! Angle between the line from robot to waypoint and the waypoints orientation (only used for output?)
    double angle = MathHelper::AngleClamp(std::atan2(delta(1), delta(0)) - std::atan2(target_dir(1), target_dir(0)));

    ROS_DEBUG_STREAM_THROTTLE(1, "angle = " << angle);

    bool reached_point = delta.dot(target_dir) < 0;  // done, if angle is greater than 90°?!


    return reached_point;
}

bool RobotController_Omnidrive_Pid::setCommand(double e_direction, double e_rotation, float speed)
{
    BehaviouralPathDriver::Options path_driver_opt = path_driver_->getOptions();
    BehaviourDriveBase* behaviour = ((BehaviourDriveBase*) path_driver_->getActiveBehaviour());

    setStatus(path_msgs::FollowPathResult::MOTION_STATUS_MOVING);

    double errors[] = {e_direction, e_rotation};
    vector<double> deltas;
    if (!pids_.execute(errors, deltas)) {
        // Nothing to do
        return false;
    }
    double delta_dir = deltas[DIRECTION];
    double delta_rot = deltas[ORIENTATION];
    ROS_DEBUG("PID-Direction: error = %g,\t delta = %g", e_direction, delta_dir);
    ROS_DEBUG("PID-Rotation:  error = %g,\t delta = %g", e_rotation, delta_rot);

    visualizer_->drawSteeringArrow(14, path_driver_->getSlamPoseMsg(), delta_dir, 0.0, 1.0, 1.0);


    bool collision = false;
    //TODO: VFH here


    double steer = std::abs(delta_dir);
    //ROS_DEBUG_STREAM("dir=" << dir_sign_ << ", steer=" << steer);
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


    cmd_.direction_angle += delta_dir;
    cmd_.direction_angle = MathHelper::AngleClamp(cmd_.direction_angle);
    //ROS_DEBUG("CMD-Direction = %g", cmd_.direction_angle);
    cmd_.rotation = delta_rot;


    collision |= behaviour->isCollision(predictSmoothedDirectionOfMovementAngle());

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
//    double dt = options_.dead_time_;
//    double v  = getFilteredSpeed();
//    double ds = dt*v;

    //TODO: do real prediction here?

    return path_driver_->getSlamPose().head<2>();
}

Eigen::Vector2d RobotController_Omnidrive_Pid::predictDirectionOfMovement()
{
    //TODO: more sophisticated prediction

    Vector2d direction;

    if (!has_last_position_) {
        // This is the first call, we can say nothing about the direction of movement.
        // -> assume we are moving towards the next waypoint
        direction = next_wp_local_.head<2>();

        last_slam_pos_update_time_ = ros::Time(0); // force update
    }
    else {
        // transform last position to robot frame
        geometry_msgs::PoseStamped last_pos_msg;
        last_pos_msg.pose.position.x = last_position_direction_update_.x();
        last_pos_msg.pose.position.y = last_position_direction_update_.y();
        last_pos_msg.pose.orientation.w = 1;

        Vector3d last_position;
        if ( !path_driver_->getNode()->transformToLocal(last_pos_msg, last_position) ) {
            setStatus(path_msgs::FollowPathResult::MOTION_STATUS_SLAM_FAIL);
            throw new BehaviourEmergencyBreak(*path_driver_);
        }

        // calculate direction of movement (current_pos - last_pos, where current_pos = 0)
        direction = -last_position.head<2>();
    }

    // only update every 0.3 seconds
    if (last_slam_pos_update_time_ < ros::Time::now() - ros::Duration(0.3)) {
        last_position_direction_update_ = path_driver_->getSlamPose().head<2>();
        last_slam_pos_update_time_ = ros::Time::now();
        has_last_position_ = true;
    }

    return direction;
}

double RobotController_Omnidrive_Pid::predictSmoothedDirectionOfMovementAngle()
{
    //TODO: I'm not so happy with this, it is rather a dirty hack to make obstacle detection stable even when the robot
    //      makes slight sideways movements. I think, there must be a better, more reliable solution...

    // Only update, if the robot has moved at least a certain distance.
    Vector2d current_pos = predictPosition();
    double driven_dist = (last_position_smoothed_direction_update_ - current_pos).norm();

    if (has_last_position_smoothed_ && driven_dist > 0.1) {
        // update
        Vector2d direction = predictDirectionOfMovement();

        const float r = 0.7;
        smoothed_direction_ = r*smoothed_direction_ + (1-r)*direction;

        last_position_smoothed_direction_update_ = current_pos;
        has_last_position_smoothed_ = true;
    }

    return atan2(smoothed_direction_[1], smoothed_direction_[0]);
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

    Vector2d pred_position = predictPosition();

    return -target_line.GetSignedDistance(pred_position);
}

double RobotController_Omnidrive_Pid::calculateDirectionError()
{
    Vector2d vec_to_wp = next_wp_local_.head<2>();

    Vector2d direction = predictDirectionOfMovement();
    double dir_angle = atan2(direction[1], direction[0]);

    // angle between the direction to the waypoint and the actual direction of movement.
    double angle = atan2(vec_to_wp(1), vec_to_wp(0)) - dir_angle;
    angle = MathHelper::AngleClamp(angle);

    return angle;
}

double RobotController_Omnidrive_Pid::calculateDistanceToWaypoint()
{
    Vector2d pred_pos = predictPosition();
    Vector2d distance = next_wp_local_.head<2>() - pred_pos;

    //NOTE: this is wrong?! - only intended for use in Approach Turning Point, where x ~ 0
    if(std::abs(distance(1)) < 0.1) {
        return 0;
    }

    return distance(1);
}
