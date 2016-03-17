// HEADER
#include <path_follower/legacy/robotcontroller_kinematic_SSG.h>

// THIRD PARTY
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


// PROJECT
#include <path_follower/pathfollower.h>
#include <path_follower/utils/cubic_spline_interpolation.h>
#include "../alglib/interpolation.h"
#include <utils_general/MathHelper.h>
#include <cmath>

// SYSTEM
#include <deque>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <boost/algorithm/clamp.hpp>

using namespace Eigen;


RobotController_Kinematic_SSG::RobotController_Kinematic_SSG(PathFollower *path_driver):
    RobotController_Interpolation(path_driver),
    cmd_(this),
    vn_(0),
    Ts_(0.02),
    ind_(0),
    proj_ind_(0),
    xe_(0),
    ye_(0),
    Vl_(0),
    Vr_(0),
    curv_sum_(1e-3),
    distance_to_goal_(1e6),
    distance_to_obstacle_(1)
{
    laser_sub_front_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan/front/filtered", 10,
                                                             &RobotController_Kinematic_SSG::laserFront, this);
    laser_sub_back_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan/back/filtered", 10,
                                                            &RobotController_Kinematic_SSG::laserBack, this);

    wheel_velocities_ = nh_.subscribe<std_msgs::Float64MultiArray>("/wheel_velocities", 10,
                                                                   &RobotController_Kinematic_SSG::WheelVelocities, this);
}

void RobotController_Kinematic_SSG::stopMotion()
{

    cmd_.speed = 0;
    cmd_.direction_angle = 0;
    cmd_.rotation = 0;

    MoveCommand mcmd = cmd_;
    publishMoveCommand(mcmd);
}

void RobotController_Kinematic_SSG::initialize()
{
    RobotController_Interpolation::initialize();

    //reset the index of the current point on the path
    ind_ = 0;

    //reset the index of the orthogonal projection
    proj_ind_ = 0;

    // desired velocity
    vn_ = std::min(path_driver_->getOptions().max_velocity(), velocity_);
    ROS_WARN_STREAM("velocity_: " << velocity_ << ", vn: " << vn_);


}

void RobotController_Kinematic_SSG::laserFront(const sensor_msgs::LaserScanConstPtr &scan)
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

void RobotController_Kinematic_SSG::laserBack(const sensor_msgs::LaserScanConstPtr &scan)
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

void RobotController_Kinematic_SSG::WheelVelocities(const std_msgs::Float64MultiArray::ConstPtr& array)
{
    double frw = array->data[0];
    double flw = array->data[1];
    double brw = array->data[2];
    double blw = array->data[3];

    Vl_ = (flw + blw)/2.0;
    Vr_ = (frw + brw)/2.0;
}

//TODO: work with the obstacle map!!!
void RobotController_Kinematic_SSG::findMinDistance()
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

void RobotController_Kinematic_SSG::start()
{
    path_driver_->getCoursePredictor().reset();
}

void RobotController_Kinematic_SSG::reset()
{
    RobotController_Interpolation::reset();
}

void RobotController_Kinematic_SSG::calculateMovingDirection()
{
    const std::vector<Waypoint> subp = path_->getCurrentSubPath();

    const double theta_0 = subp[0].orientation;
    Eigen::Vector2d looking_dir_normalized(std::cos(theta_0), std::sin(theta_0));
    Eigen::Vector2d delta(subp[1].x - subp[0].x, subp[1].y - subp[0].y);
    const double theta_diff = std::acos(delta.dot(looking_dir_normalized) / delta.norm());

    // decide whether to drive forward or backward
    if (theta_diff > M_PI_2 || theta_diff < -M_PI_2) {
        setDirSign(-1.f);
    } else {
        setDirSign(1.f);
    }
}

void RobotController_Kinematic_SSG::setPath(Path::Ptr path)
{
    RobotController_Interpolation::setPath(path);

    calculateMovingDirection();
}

RobotController::MoveCommandStatus RobotController_Kinematic_SSG::computeMoveCommand(MoveCommand *cmd)
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


    // check for the subpaths, and see if the goal is reached
    if((ind_ == path_interpol.n()-1) & (xe_ > 0.0)) {
        path_->switchToNextSubPath();
        // check if we reached the actual goal or just the end of a subpath
        if (path_->isDone()) {

            cmd_.speed = 0;
            cmd_.direction_angle = 0;
            cmd_.rotation = 0;

            *cmd = cmd_;

            double distance_to_goal_eucl = hypot(x_meas - path_interpol.p(path_interpol.n()-1),
                                          y_meas - path_interpol.q(path_interpol.n()-1));

            ROS_INFO("Final positioning error: %f m", distance_to_goal_eucl);

            return RobotController::MoveCommandStatus::REACHED_GOAL;

        } else {

            ROS_INFO("Next subpath...");
            // interpolate the next subpath
            try {
                path_interpol.interpolatePath(path_);
                publishInterpolatedPath();
            } catch(const alglib::ap_error& error) {
                throw std::runtime_error(error.msg);
            }
            // recalculate the driving direction
            calculateMovingDirection();
        }
    }


    //find the orthogonal projection to the curve and extract the corresponding index

    double dist = 0;
    double orth_proj = std::numeric_limits<double>::max();

    for (unsigned int i = proj_ind_; i < path_interpol.n(); i++){

        dist = hypot(x_meas - path_interpol.p(i), y_meas - path_interpol.q(i));
        if(dist < orth_proj){

            orth_proj = dist;
            proj_ind_ = i;

        }

    }
    //***//


    ///calculate the control for the current point on the path

    //robot direction angle in path coordinates
    double theta_e = MathHelper::AngleDelta(path_interpol.theta_p(ind_), theta_meas);

    //robot position vector module
    double r = hypot(x_meas - path_interpol.p(ind_), y_meas - path_interpol.q(ind_));

    //robot position vector angle in world coordinates
    double theta_r = atan2(y_meas - path_interpol.q(ind_), x_meas - path_interpol.p(ind_));

    //robot position vector angle in path coordinates
    double delta_theta = MathHelper::AngleDelta(path_interpol.theta_p(ind_), theta_r);

    //current robot position in path coordinates
    xe_ = r * cos(delta_theta);
    ye_ = r * sin(delta_theta);

    ///***///


    ///Check the driving direction, and set the complementary angle in path coordinates, if driving backwards

    if (getDirSign() < 0.0) {
        theta_e = MathHelper::NormalizeAngle(M_PI + theta_e);
        ROS_WARN_THROTTLE(1, "Driving backwards...");
    }

    ///***///


    ///Calculate the parameters for the exponential speed control

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
    distance_to_goal_ = path_interpol.s(path_interpol.n()-1) - path_interpol.s(proj_ind_);

    //get the robot's current angular velocity
    double angular_vel = path_driver_->getVelocity().angular.z;
    ///***///


//    ///Lyapunov-curvature speed control - ideal unicycle

//    //Lyapunov function as a measure of the path following error
//    double v = vn_;
//    double V1 = 0.5*(std::pow(xe_,2) + std::pow(ye_,2)) + (0.5/opt_.gamma())*std::pow((theta_e - delta_),2);

//    //use v/2 as the minimum speed, and allow larger values when the error is small
//    if (V1 >= opt_.epsilon()) v = 0.5*v;

//    else if (V1 < opt_.epsilon()) v = v/(1 + opt_.b()*std::abs(path_interpol.curvature(ind_)));

//    ///***///

    ///Lyapunov-curvature speed control - skid steering

    double v = vn_;
    double V1 = 1.0/2.0*(std::pow(xe_,2) + std::pow(ye_,2) + 1.0/opt_.lambda()*std::pow(theta_e,2));

    if(angular_vel >= 0){

        if(V1 >= opt_.epsilon()){
            v = (-opt_.alpha_r()*opt_.y_ICR_l()*vn_)/(opt_.y_ICR_r() - opt_.y_ICR_l());
        }
        else if(V1 < opt_.epsilon()){
            v = (opt_.alpha_r()*vn_)/(1 + std::fabs(opt_.y_ICR_r()*path_interpol.curvature(ind_)));
        }
    }
    else if(angular_vel < 0){

        if(V1 >= opt_.epsilon()){
            v = (opt_.alpha_l()*opt_.y_ICR_r()*vn_)/(opt_.y_ICR_r() - opt_.y_ICR_l());
        }
        else if(V1 < opt_.epsilon()){
            v = (opt_.alpha_l()*vn_)/(1 + std::fabs(opt_.y_ICR_l()*path_interpol.curvature(ind_)));
        }
    }





    ///Calculate the next point on the path

    double omega_meas = path_driver_->getVelocity().angular.z;
    double v_meas = path_driver_->getVelocity().linear.x;

    double s_old = path_interpol.s_new();

    //calculate the speed of the "virtual vehicle"
    double s_prim_tmp = v_meas * cos(theta_e) + opt_.x_ICR()*omega_meas*sin(theta_e) + opt_.k1() * xe_;
    path_interpol.set_s_prim(s_prim_tmp > 0 ? s_prim_tmp : 0);

    //approximate the first derivative and calculate the next point
    double s_temp = Ts_*path_interpol.s_prim() + s_old;
    path_interpol.set_s_new(s_temp > 0 ? s_temp : 0);

    ///***///


    ///Direction control

    cmd_.direction_angle = 0;

    //omega_m = theta_e_prim + curv*s_prim
    double omega = opt_.lambda()*ye_/theta_e*(opt_.x_ICR()*omega_meas*cos(theta_e) - v_meas*sin(theta_e))
            - opt_.k2()*theta_e + path_interpol.curvature(ind_)*path_interpol.s_prim();


    omega = boost::algorithm::clamp(omega, -opt_.max_angular_velocity(), opt_.max_angular_velocity());
    cmd_.rotation = omega;

    ///***///


    ///Exponential speed control

    //ensure valid values
    if (distance_to_obstacle_ == 0 || !std::isfinite(distance_to_obstacle_)) distance_to_obstacle_ = 1e-10;
    if (distance_to_goal_ == 0 || !std::isfinite(distance_to_goal_)) distance_to_goal_ = 1e-10;

    double exponent = opt_.k_curv()*fabs(curv_sum_)
            + opt_.k_w()*fabs(angular_vel)
            + opt_.k_o()/distance_to_obstacle_
            + opt_.k_g()/distance_to_goal_;

    //TODO: consider the minimum excitation speed
    v = v * exp(-exponent);

    cmd_.speed = getDirSign()*std::max((double)path_driver_->getOptions().min_velocity(), fabs(v));

    ///***///



    ///plot the moving reference frame together with position vector and error components

    if (visualizer_->MarrayhasSubscriber()) {
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

    if(old_ind != ind_) {
        path_->fireNextWaypointCallback();
    }

    ///***///


    if (visualizer_->hasSubscriber()) {
        visualizer_->drawSteeringArrow(1, path_driver_->getRobotPoseMsg(), cmd_.direction_angle, 0.2, 1.0, 0.2);
    }

    ///***///

        *cmd = cmd_;

        return MoveCommandStatus::OKAY;
}

void RobotController_Kinematic_SSG::publishMoveCommand(const MoveCommand &cmd) const
{
    geometry_msgs::Twist msg;
    msg.linear.x  = cmd.getVelocity();
    msg.linear.y  = 0;
    msg.angular.z = cmd.getRotationalVelocity();

    cmd_pub_.publish(msg);
}

