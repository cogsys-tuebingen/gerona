#include <path_follower/legacy/robotcontroller_ackermann_pid.h>

/// STL
#include <sstream>

/// ROS
#include <ros/ros.h>

/// PROJECT
#include <path_follower/pathfollower.h>
#include <path_follower/legacy/behaviours.h>
#include <utils_general/MathHelper.h>

using namespace Eigen;
using namespace std;


namespace {
double sign(double value) {
    if (value < 0) return -1.0;
    if (value > 0) return 1.0;
    return 0.0;
}
}


RobotController_Ackermann_Pid::RobotController_Ackermann_Pid(ros::Publisher &cmd_publisher,
                                                             PathFollower *path_driver,
                                                             VectorFieldHistogram *vfh):
    RobotController(cmd_publisher, path_driver),
    vfh_(vfh)
{
    configure();

    visualizer_ = Visualizer::getInstance();
}

void RobotController_Ackermann_Pid::configure()
{
    ros::NodeHandle nh("~");

    nh.param( "dead_time", options_.dead_time_, 0.10 );
    nh.param( "l", options_.l_, 0.38 );

    double ta, kp, ki, i_max, delta_max, e_max;
    nh.param( "pid/ta", ta, 0.03 );
    nh.param( "pid/kp", kp, 1.5 );
    nh.param( "pid/ki", ki, 0.001 );
    nh.param( "pid/i_max", i_max, 0.0 );
    nh.param( "pid/delta_max", delta_max, 30.0 );
    nh.param( "pid/e_max", e_max, 0.10 );

    pid_.configure( kp, ki, i_max, M_PI*delta_max/180.0, e_max, 0.5, ta );
}

bool RobotController_Ackermann_Pid::setCommand(double error, float speed)
{
    PathFollower::Options path_driver_opt = path_driver_->getOptions();

    double delta_f_raw = 0;

    setStatus(path_msgs::FollowPathResult::MOTION_STATUS_MOVING);

    if (!pid_.execute( error, delta_f_raw)) {
        // Nothing to do
        return false;
    }
    ROS_DEBUG("PID: error = %g, df = %g", error, delta_f_raw);

    visualizer_->drawSteeringArrow(14, path_driver_->getRobotPoseMsg(), delta_f_raw, 0.0, 1.0, 1.0);

    double threshold = 5.0;
    double threshold_max_distance = 3.5 /*m*/;

    double distance_to_goal = path_.current_path->back().distanceTo(path_.nextWaypoint());
    double threshold_distance = std::min(threshold_max_distance,
                                         std::max((double) path_driver_opt.collision_box_min_length(), distance_to_goal));

    double delta_f = delta_f_raw;
    bool collision = false;

    if (vfh_ != 0) {
        if(!vfh_->isReady()) {
            ROS_WARN_THROTTLE(1, "Not using VFH, not ready yet! (Maybe obstacle map not published?)");
            //delta_f = delta_f_raw;
        } else {
            vfh_->create(threshold_distance, threshold);
            collision = !vfh_->adjust(delta_f_raw, threshold, delta_f);
            vfh_->visualize(delta_f_raw, threshold);
        }
    }

    visualizer_->drawSteeringArrow(14, path_driver_->getRobotPoseMsg(), delta_f, 0.0, 1.0, 1.0);


    double steer = std::abs(delta_f);
    ROS_DEBUG_STREAM("dir=" << dir_sign_ << ", steer=" << steer);
    if(steer > path_driver_opt.steer_slow_threshold()) {
        ROS_WARN_STREAM_THROTTLE(2, "slowing down");
        speed *= 0.5;
    }

    // make sure, the speed is in the allowed range
    if (speed < path_driver_opt.min_velocity()) {
        speed = path_driver_opt.min_velocity();
        ROS_WARN_THROTTLE(5, "Velocity is below minimum. It is set to minimum velocity.");
    } else if (speed > path_driver_opt.max_velocity()) {
        speed = path_driver_opt.max_velocity();
        ROS_WARN_THROTTLE(5, "Velocity is above maximum. Reduce to maximum velocity.");
    }

    cmd_.steer_front = dir_sign_ * delta_f;
    cmd_.steer_back = 0;

    // no laser backward, so do not check when driving backwards.
    if (dir_sign_ > 0) {
        collision |= path_driver_->isObstacleAhead(calculateCourse());
    }

    if(collision) {
        ROS_WARN_THROTTLE(1, "Collision!");
        setStatus(path_msgs::FollowPathResult::MOTION_STATUS_OBSTACLE); //TODO: not so good to use result-constant if it is not finishing the action...

        stopMotion();
    } else {
        cmd_.velocity = dir_sign_ * speed;
    }

    ROS_DEBUG("Set velocity to %g", cmd_.velocity);
    return (std::abs(delta_f - delta_f_raw) > 0.05);
}

void RobotController_Ackermann_Pid::publishCommand()
{
    if (!cmd_.isValid()) {
        ROS_FATAL("Invalid Command in RobotController_Ackermann_Pid.");
        return;
    }

    //ramaxx_msgs::RamaxxMsg msg = current_command_;
    geometry_msgs::Twist msg = cmd_;
    cmd_pub_.publish(msg);

    setFilteredSpeed(cmd_.velocity);
}

void RobotController_Ackermann_Pid::stopMotion()
{
    cmd_.velocity = 0.0;
    cmd_.steer_front = 0.0;
    cmd_.steer_back= 0.0;

    publishCommand();
}

void RobotController_Ackermann_Pid::initOnLine()
{
    pid_.reset();
}

void RobotController_Ackermann_Pid::behaveOnLine()
{
    dir_sign_ = sign(next_wp_local_.x());

    // Calculate target line from current to next waypoint (if there is any)
    double e_distance = calculateLineError();
    double e_angle = calculateAngleError();

    double e_combined = e_distance + e_angle;
    ROS_DEBUG("OnLine: e_dist = %g, e_angle = %g  ==>  e_comb = %g", e_distance, e_angle, e_combined);

    // draw steer front
    if (visualizer_->hasSubscriber()) {
        visualizer_->drawSteeringArrow(1, path_driver_->getRobotPoseMsg(), e_angle, 0.2, 1.0, 0.2);
        visualizer_->drawSteeringArrow(2, path_driver_->getRobotPoseMsg(), e_distance, 0.2, 0.2, 1.0);
        visualizer_->drawSteeringArrow(3, path_driver_->getRobotPoseMsg(), e_combined, 1.0, 0.2, 0.2);
    }

    float speed = velocity_;

    // TODO: better speed control
    //       - backwards not slower, but lower max speed
    //       - slower when close to goal, similar to ApproachGoal
    if(dir_sign_ < 0) {
        speed *= 0.5;
    }

    if(setCommand(e_combined, speed)) {
        setStatus(path_msgs::FollowPathResult::MOTION_STATUS_MOVING);
        throw new BehaviourAvoidObstacle(*path_driver_);
    }
}

void RobotController_Ackermann_Pid::behaveAvoidObstacle()
{
    dir_sign_ = sign(next_wp_local_.x());

    // Calculate target line from current to next waypoint (if there is any)
    double e_distance = calculateLineError();
    double e_angle = calculateAngleError();

    double e_combined = e_distance + e_angle;

    // draw steer front
    if (visualizer_->hasSubscriber()) {
        visualizer_->drawSteeringArrow(1, path_driver_->getRobotPoseMsg(), e_angle, 0.2, 1.0, 0.2);
        visualizer_->drawSteeringArrow(2, path_driver_->getRobotPoseMsg(), e_distance, 0.2, 0.2, 1.0);
        visualizer_->drawSteeringArrow(3, path_driver_->getRobotPoseMsg(), e_combined, 1.0, 0.2, 0.2);
    }

    float speed = velocity_;

    if(dir_sign_ < 0) {
        speed *= 0.5;
    }

    if(!setCommand(e_combined, speed)) {
        setStatus(path_msgs::FollowPathResult::MOTION_STATUS_MOVING);
        throw new BehaviourOnLine(*path_driver_);
    }
}

void RobotController_Ackermann_Pid::initApproachTurningPoint()
{
    atp_step_ = 0;
}

bool RobotController_Ackermann_Pid::behaveApproachTurningPoint()
{
    // CHECK IF DONE
    float dir_sign = sign(next_wp_local_.x());
    if(atp_step_++ > 0 && dir_sign != getDirSign()) {
        return true;
    }
    setDirSign(dir_sign);


    // Calculate target line from current to next waypoint (if there is any)
    double e_distance = calculateDistanceError();
    double e_angle = calculateAngleError();

    double e_combined = e_distance + e_angle;
    ROS_DEBUG("Approach: e_dist = %g, e_angle = %g  ==>  e_comb = %g", e_distance, e_angle, e_combined);

    if (visualizer_->hasSubscriber()) {
        visualizer_->drawCircle(2, ((geometry_msgs::Pose) path_.nextWaypoint()).position, 0.5, "/map", "turning point", 1, 1, 1);

        // draw steer front
        visualizer_->drawSteeringArrow(1, path_driver_->getRobotPoseMsg(), e_angle, 0.2, 1.0, 0.2);
        visualizer_->drawSteeringArrow(2, path_driver_->getRobotPoseMsg(), e_distance, 0.2, 0.2, 1.0);
        visualizer_->drawSteeringArrow(3, path_driver_->getRobotPoseMsg(), e_combined, 1.0, 0.2, 0.2);
    }

    float distance = std::sqrt(next_wp_local_.dot(next_wp_local_));
    float velocity = std::max(0.1f + distance / 2.0f, path_driver_->getOptions().min_velocity());

    setCommand(e_combined, velocity);

    return false;
}

void RobotController_Ackermann_Pid::predictPose(Vector2d &front_pred, Vector2d &rear_pred)
{
    double dt = options_.dead_time_;
    double deltaf = cmd_.steer_front;
    double deltar = cmd_.steer_back;
    double v = 2*getFilteredSpeed();

    double beta = std::atan(0.5*(std::tan(deltaf)+std::tan(deltar)));
    double ds = v*dt;
    double dtheta = ds*std::cos(beta)*(std::tan(deltaf)-std::tan(deltar))/options_.l_;
    double thetan = dtheta; //TODO <- why this ???
    double yn = ds*std::sin(dtheta*0.5+beta*0.5);
    double xn = ds*std::cos(dtheta*0.5+beta*0.5);

    ROS_DEBUG("predict pose: dt = %g, deltaf = %g, deltar = %g, v = %g, beta = %g, ds = %g, dtheta = %g, yn = %g, xn = %g",
              dt, deltaf, deltar, v, beta, ds, dtheta, yn, xn);
    // output when nan-values occured (hopefully fixed with commit 8bad8fd)
    // step 1: dt = 0.1, deltaf = 9.75024e+199, deltar = 4.26137e+257, v = 0,   beta = 0.308293, ds = 0,   dtheta = 0,   yn = 0,    xn = 0
    // step 2: dt = 0.1, deltaf = 9.75024e+199, deltar = 4.26137e+257, v = inf, beta = 0.308293, ds = inf, dtheta = inf, yn = -nan, xn = -nan

    front_pred[0] = xn+cos(thetan)*options_.l_/2.0;
    front_pred[1] = yn+sin(thetan)*options_.l_/2.0;
    rear_pred[0] = xn-cos(thetan)*options_.l_/2.0;
    rear_pred[1] = yn-sin(thetan)*options_.l_/2.0;

    ROS_DEBUG_STREAM("predict pose. front: " << front_pred << ", rear: " << rear_pred);
}

double RobotController_Ackermann_Pid::calculateCourse()
{
    return cmd_.steer_front;
}

double RobotController_Ackermann_Pid::calculateLineError()
{
    geometry_msgs::PoseStamped followup_next_wp_map;
    followup_next_wp_map.header.stamp = ros::Time::now();

    if(path_.wp_idx + 1 == path_.current_path->size()) {
        followup_next_wp_map.pose = path_.getWaypoint(path_.wp_idx - 1);
    } else {
        followup_next_wp_map.pose = path_.getWaypoint(path_.wp_idx + 1);
    }

    Line2d target_line;
    Vector3d followup_next_wp_local;
    if (!path_driver_->transformToLocal( followup_next_wp_map, followup_next_wp_local)) {
        setStatus(path_msgs::FollowPathResult::MOTION_STATUS_INTERNAL_ERROR);
        throw new BehaviourEmergencyBreak(*path_driver_);
    }
    target_line = Line2d( next_wp_local_.head<2>(), followup_next_wp_local.head<2>());
    visualizer_->visualizeLine(target_line);

    Vector2d main_carrot, alt_carrot, front_pred, rear_pred;
    predictPose(front_pred, rear_pred);
    if(dir_sign_ >= 0) {
        main_carrot = front_pred;
        alt_carrot = rear_pred;
    } else {
        main_carrot = rear_pred;
        alt_carrot = front_pred;
    }

    if (visualizer_->hasSubscriber()) {
        visualizeCarrot(main_carrot, 0, 1.0,0.0,0.0);
        visualizeCarrot(alt_carrot, 1, 0.0,0.0,0.0);
    }

    return -target_line.GetSignedDistance(main_carrot) - 0.25 * target_line.GetSignedDistance(alt_carrot);
}

double RobotController_Ackermann_Pid::calculateDistanceError()
{
    Vector2d main_carrot, alt_carrot, front_pred, rear_pred;

    predictPose(front_pred, rear_pred);
    if(dir_sign_ >= 0) {
        main_carrot = front_pred;
        alt_carrot = rear_pred;
    } else {
        main_carrot = rear_pred;
        alt_carrot = front_pred;
    }

    if (visualizer_->hasSubscriber()) {
        visualizeCarrot(main_carrot, 0, 1.0,0.0,0.0);
        visualizeCarrot(alt_carrot, 1, 0.0,0.0,0.0);
    }

    Vector2d delta = next_wp_local_.head<2>() - main_carrot;

    if(std::abs(delta(1)) < 0.1) {
        return 0;
    }

    return delta(1);
}

void RobotController_Ackermann_Pid::visualizeCarrot(const Vector2d &carrot, int id, float r, float g, float b)
{
    geometry_msgs::PoseStamped carrot_local;
    carrot_local.pose.position.x = carrot[0];
    carrot_local.pose.position.y = carrot[1];

    carrot_local.pose.orientation = tf::createQuaternionMsgFromYaw(0);
    geometry_msgs::PoseStamped carrot_map;
    if (path_driver_->transformToGlobal(carrot_local, carrot_map)) {
        visualizer_->drawMark(id, carrot_map.pose.position, "prediction", r,g,b);
    }
}
