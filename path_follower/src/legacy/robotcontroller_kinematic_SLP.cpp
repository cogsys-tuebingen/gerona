// HEADER
#include <path_follower/legacy/robotcontroller_kinematic_SLP.h>

// THIRD PARTY
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// PROJECT
#include <path_follower/pathfollower.h>
#include <path_follower/utils/cubic_spline_interpolation.h>
#include <utils_general/MathHelper.h>
#include <cmath>

// SYSTEM
#include <deque>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;


RobotController_Kinematic_SLP::RobotController_Kinematic_SLP(PathFollower *path_driver):
    RobotController_Interpolation(path_driver),
    cmd_(this),
    view_direction_(LookInDrivingDirection),
    vn_(0),
    delta_(0),
    Ts_(0.02),
    ind_(0),
    sign_v_(0),
    xe_(0),
    ye_(0),
    curv_sum_(1e-3),
    distance_to_goal_(1e-3),
    distance_to_obstacle_(1e-3)
{
    look_at_cmd_sub_ = nh_.subscribe<std_msgs::String>("/look_at/cmd", 10,
                                                       &RobotController_Kinematic_SLP::lookAtCommand, this);
    look_at_sub_ = nh_.subscribe<geometry_msgs::PointStamped>("/look_at", 10,
                                                              &RobotController_Kinematic_SLP::lookAt, this);

    laser_sub_front_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan/front/filtered", 10,
                                                             &RobotController_Kinematic_SLP::laserFront, this);
    laser_sub_back_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan/back/filtered", 10,
                                                            &RobotController_Kinematic_SLP::laserBack, this);

    lookInDrivingDirection();

}

void RobotController_Kinematic_SLP::stopMotion()
{

    cmd_.speed = 0;
    cmd_.direction_angle = 0;
    cmd_.rotation = 0;

    MoveCommand mcmd = cmd_;
    publishMoveCommand(mcmd);
}

void RobotController_Kinematic_SLP::lookAtCommand(const std_msgs::StringConstPtr &cmd)
{
    const std::string& command = cmd->data;

    if(command == "reset" || command == "view") {
        lookInDrivingDirection();
    } else if(command == "keep") {
        keepHeading();
    } else if(command == "rotate") {
        rotate();
    }
}

void RobotController_Kinematic_SLP::initialize()
{
    RobotController_Interpolation::initialize();

    //reset the index of the current point on the path
    ind_ = 0;

    // desired velocity
    vn_ = std::min(path_driver_->getOptions().max_velocity(), velocity_);
    ROS_WARN_STREAM("velocity_: " << velocity_ << ", vn: " << vn_);
}

void RobotController_Kinematic_SLP::lookAt(const geometry_msgs::PointStampedConstPtr &look_at)
{
    look_at_ = look_at->point;
    view_direction_ = LookAtPoint;
}

void RobotController_Kinematic_SLP::laserFront(const sensor_msgs::LaserScanConstPtr &scan)
{
    ranges_front_.clear();
    for(std::size_t i = 0, total = scan->ranges.size(); i < total; ++i) {
        float range = scan->ranges[i];
        if(range > scan->range_min && range < scan->range_max) {
            ranges_front_.push_back(range);
        }
    }
    findMinDistance();
}

void RobotController_Kinematic_SLP::laserBack(const sensor_msgs::LaserScanConstPtr &scan)
{
    ranges_back_.clear();
    for(std::size_t i = 0, total = scan->ranges.size(); i < total; ++i) {
        float range = scan->ranges[i];
        if(range > scan->range_min && range < scan->range_max) {
            ranges_back_.push_back(range);
        }
    }
    findMinDistance();
}

void RobotController_Kinematic_SLP::findMinDistance()
{
    std::vector<float> ranges;
    ranges.insert(ranges.end(), ranges_front_.begin(), ranges_front_.end());
    ranges.insert(ranges.end(), ranges_back_.begin(), ranges_back_.end());
    std::sort(ranges.begin(), ranges.end());

    if(ranges.size() <= 7) {
       distance_to_obstacle_ = 0;
        return;
    }

    distance_to_obstacle_ = ranges[0];

}

void RobotController_Kinematic_SLP::keepHeading()
{
    view_direction_ = KeepHeading;
}

void RobotController_Kinematic_SLP::rotate()
{
    view_direction_ = Rotate;
}

void RobotController_Kinematic_SLP::lookInDrivingDirection()
{
    view_direction_ = LookInDrivingDirection;
}

void RobotController_Kinematic_SLP::start()
{
    path_driver_->getCoursePredictor().reset();
}

RobotController::MoveCommandStatus RobotController_Kinematic_SLP::computeMoveCommand(MoveCommand *cmd)
{
    // omni drive can rotate.
    *cmd = MoveCommand(true);

    if(path_interpol.n() < 2) {
        ROS_ERROR("[Line] path is too short (N = %d)", (int) path_interpol.n());

        stopMotion();
        return MoveCommandStatus::REACHED_GOAL;
    }


    /// get the pose as pose(0) = x, pose(1) = y, pose(2) = theta
    Eigen::Vector3d current_pose = path_driver_->getRobotPose();

    double x_meas = current_pose[0];
    double y_meas = current_pose[1];
    double theta_meas = current_pose[2];
    ///***///


    ///calculate the control for the current point on the path

    //robot direction angle in path coordinates
    double theta_e = theta_meas - path_interpol.theta_p(ind_);

    //robot position vector module
    double r = hypot(x_meas - path_interpol.p(ind_), y_meas - path_interpol.q(ind_));

    //robot position vector angle in world coordinates
    double theta_r = atan2(y_meas - path_interpol.q(ind_), x_meas - path_interpol.p(ind_));

    //robot position vector angle in path coordinates
    double delta_theta = MathHelper::AngleClamp(theta_r - path_interpol.theta_p(ind_));

    //current robot position in path coordinates
    xe_ = r * cos(delta_theta);
    ye_ = r * sin(delta_theta);

    ///***///


    ///Calculate the parameters for exponential speed control

    //calculate the curvature, and stop when the look-ahead distance is reached (w.r.t. orthogonal projection)
    double s_cum_sum = 0;
    curv_sum_ = 1e-10;

    for (unsigned int i = ind_ + 1; i < path_interpol.n(); i++){

        s_cum_sum = path_interpol.s(i) - path_interpol.s(ind_);
        curv_sum_ += fabs(path_interpol.curvature(i));

        if(s_cum_sum - opt_.look_ahead_dist() >= 0){
            break;
        }
    }

    //calculate the distance from the orthogonal projection to the goal, w.r.t. path
    distance_to_goal_ = path_interpol.s(path_interpol.n()-1) - path_interpol.s(ind_);

    //get the robot's current angular velocity
    double angular_vel = path_driver_->getVelocity().angular.z;
    ///***///

    ///Calculate the delta_ and its derivative

    double delta_old = delta_;

    delta_ = -sign_v_*opt_.theta_a()*tanh(ye_);

    double delta_prim = (delta_ - delta_old)/Ts_;
    ///***///


    ///Exponential speed control

    //ensure valid values
    if(distance_to_obstacle_ == 0 || !std::isfinite(distance_to_obstacle_)) distance_to_obstacle_ = 1e-10;
    if(distance_to_goal_ == 0 || !std::isfinite(distance_to_goal_)) distance_to_goal_ = 1e-10;

    double exponent = opt_.k_curv()*fabs(curv_sum_)
            + opt_.k_w()*fabs(angular_vel)
            + opt_.k_o()/distance_to_obstacle_
            + opt_.k_g()/distance_to_goal_;

    //TODO: consider the minimum excitation speed
    double v = std::max(0.4,vn_*exp(-exponent));

    ///***///


    ///Lyapunov-curvature speed control

    //Lyapunov function as a measure of the path following error
    double V1 = 0.5*(std::pow(xe_,2) + std::pow(ye_,2)) + (0.5/opt_.gamma())*std::pow((theta_e - delta_),2);

    //use v/2 as the minimum speed, and allow larger values when the error is small
    if(V1 >= opt_.epsilon()) v = 0.5*v;

    else if(V1 < opt_.epsilon()) v = v/(1 + opt_.b()*std::abs(path_interpol.curvature(ind_)));


    cmd_.speed = v;

    ///***///


    ///Calculate the next point on the path

    double s_old = path_interpol.s_new();

    //calculate the speed of the "virtual vehicle"
    path_interpol.set_s_prim(v * cos(theta_e) + opt_.k1() * xe_);

    //approximate the first derivative and calculate the next point
    double s_temp = Ts_*path_interpol.s_prim() + s_old;
    path_interpol.set_s_new(s_temp > 0 ? s_temp : 0);

    ///***///


    ///Direction control

    cmd_.direction_angle = 0;

    //omega_m = theta_prim + curv*s_prim
    cmd_.rotation = delta_prim - opt_.gamma()*ye_*v*(sin(theta_e) - sin(delta_))
                    /(theta_e - delta_) - opt_.k2()*(theta_e - delta_) + path_interpol.curvature(ind_)*path_interpol.s_prim();

    ///***///

    ///Get the velocity sign

    if(v > 0) sign_v_ = 1;
    else if (v < 0) sign_v_ = -1;
    else sign_v_ = 0;

    ///***///


    ///plot the moving reference frame together with position vector and error components

    if (visualizer_->hasSubscriber()) {
        visualizer_->drawFrenetSerretFrame(0, current_pose, xe_, ye_, path_interpol.p(ind_),
                                           path_interpol.q(ind_), path_interpol.theta_p(ind_));
    }

    ///***///


    ///Calculate the index of the new point

    double s_diff = std::numeric_limits<double>::max();
    uint old_ind = ind_;

    for (unsigned int i = old_ind; i < path_interpol.n(); i++){

        double s_diff_curr = std::abs(path_interpol.s_new() - path_interpol.s(i));

        if(s_diff_curr < s_diff){

            s_diff = s_diff_curr;
            ind_ = i;

        }

    }

    ///***///


    if (visualizer_->hasSubscriber()) {
        visualizer_->drawSteeringArrow(1, path_driver_->getRobotPoseMsg(), cmd_.direction_angle, 0.2, 1.0, 0.2);
    }

    //***//


    // check for end
    double distance_to_goal_eucl = hypot(x_meas - path_interpol.p(path_interpol.n()-1),
                                         y_meas - path_interpol.q(path_interpol.n()-1));
    ROS_WARN_THROTTLE(1, "distance to goal: %f", distance_to_goal_eucl);

    if(distance_to_goal_eucl <= path_driver_->getOptions().goal_tolerance()) {
        return MoveCommandStatus::REACHED_GOAL;
    } else {
        *cmd = cmd_;

        return MoveCommandStatus::OKAY;
    }
}

void RobotController_Kinematic_SLP::publishMoveCommand(const MoveCommand &cmd) const
{
    geometry_msgs::Twist msg;
    msg.linear.x  = cmd.getVelocity();
    msg.linear.y  = 0;
    msg.angular.z = cmd.getRotationalVelocity();

    cmd_pub_.publish(msg);
}
