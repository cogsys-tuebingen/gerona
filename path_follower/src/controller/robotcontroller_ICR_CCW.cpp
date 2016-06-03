// HEADER
#include <path_follower/controller/robotcontroller_ICR_CCW.h>

// THIRD PARTY
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


// PROJECT
#include <path_follower/pathfollower.h>
#include <path_follower/utils/cubic_spline_interpolation.h>
#include <path_follower/utils/extended_kalman_filter.h>
#include "../alglib/interpolation.h"
#include <utils_general/MathHelper.h>
#include <cmath>

// SYSTEM
#include <deque>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <boost/algorithm/clamp.hpp>

using namespace Eigen;


RobotController_ICR_CCW::RobotController_ICR_CCW(PathFollower *path_driver):
    RobotController_Interpolation(path_driver),
    cmd_(this),
    vn_(0),
    Ts_(0.02),
    proj_ind_(0),
    Vl_(0),
    Vr_(0),
    curv_sum_(1e-3),
    distance_to_goal_(1e6),
    distance_to_obstacle_(1)
{
    laser_sub_front_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan/front/filtered", 10,
                                                             &RobotController_ICR_CCW::laserFront, this);
    laser_sub_back_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan/back/filtered", 10,
                                                            &RobotController_ICR_CCW::laserBack, this);

    wheel_velocities_ = nh_.subscribe<std_msgs::Float64MultiArray>("/wheel_velocities", 10,
                                                                   &RobotController_ICR_CCW::WheelVelocities, this);
}

void RobotController_ICR_CCW::stopMotion()
{

    cmd_.speed = 0;
    cmd_.direction_angle = 0;
    cmd_.rotation = 0;

    MoveCommand mcmd = cmd_;
    publishMoveCommand(mcmd);
}

void RobotController_ICR_CCW::initialize()
{
    RobotController_Interpolation::initialize();

    //reset the index of the orthogonal projection
    proj_ind_ = 0;

    // desired velocity
    vn_ = std::min(path_driver_->getOptions().max_velocity(), velocity_);
    ROS_WARN_STREAM("velocity_: " << velocity_ << ", vn: " << vn_);


}

void RobotController_ICR_CCW::laserFront(const sensor_msgs::LaserScanConstPtr &scan)
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

void RobotController_ICR_CCW::laserBack(const sensor_msgs::LaserScanConstPtr &scan)
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

void RobotController_ICR_CCW::WheelVelocities(const std_msgs::Float64MultiArray::ConstPtr& array)
{
    double flw = array->data[0];
    double frw = array->data[1];
    double brw = array->data[2];
    double blw = array->data[3];

    Vl_ = (flw + blw)/2.0;
    Vr_ = (frw + brw)/2.0;
}

//TODO: work with the obstacle map!!!
void RobotController_ICR_CCW::findMinDistance()
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

void RobotController_ICR_CCW::start()
{
    path_driver_->getCoursePredictor().reset();
}

void RobotController_ICR_CCW::reset()
{
    RobotController_Interpolation::reset();
}

void RobotController_ICR_CCW::calculateMovingDirection()
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

void RobotController_ICR_CCW::setPath(Path::Ptr path)
{
    RobotController_Interpolation::setPath(path);

    calculateMovingDirection();
}

RobotController::MoveCommandStatus RobotController_ICR_CCW::computeMoveCommand(MoveCommand *cmd)
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
    double theta_e = MathHelper::AngleDelta(path_interpol.theta_p(proj_ind_), theta_meas);

    //robot position vector module
    double r = hypot(x_meas - path_interpol.p(proj_ind_), y_meas - path_interpol.q(proj_ind_));

    //robot position vector angle in world coordinates
    double theta_r = atan2(y_meas - path_interpol.q(proj_ind_), x_meas - path_interpol.p(proj_ind_));

    //robot position vector angle in path coordinates
    double delta_theta = MathHelper::AngleDelta(path_interpol.theta_p(proj_ind_), theta_r);

    //current robot position in path coordinates
    double xe = r * cos(delta_theta);
    double ye = r * sin(delta_theta);

    ///***///


    // check for the subpaths, and see if the goal is reached
    if((proj_ind_ == path_interpol.n()-1)) {
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

    for (unsigned int i = proj_ind_ + 1; i < path_interpol.n(); i++){

        s_cum_sum = path_interpol.s(i) - path_interpol.s(proj_ind_);
        //TODO: need two types of curv_sum_, one for the exponential, the other one for the Lyapunov speed control
        //curv_sum_ += fabs(path_interpol.curvature(i));
        curv_sum_ += path_interpol.curvature(i);

        if(s_cum_sum - opt_.look_ahead_dist() >= 0){
            break;
        }
    }

    //calculate the distance from the orthogonal projection to the goal, w.r.t. path
    distance_to_goal_ = path_interpol.s(path_interpol.n()-1) - path_interpol.s(proj_ind_);
    ///***///


    ///Exponential speed control

    //get the robot's current angular velocity
    double angular_vel = path_driver_->getVelocity().angular.z;

    //ensure valid values
    if (distance_to_obstacle_ == 0 || !std::isfinite(distance_to_obstacle_)) distance_to_obstacle_ = 1e-10;
    if (distance_to_goal_ == 0 || !std::isfinite(distance_to_goal_)) distance_to_goal_ = 1e-10;

    double exponent = opt_.k_curv()*fabs(curv_sum_)
            + opt_.k_w()*fabs(angular_vel)
            + opt_.k_o()/distance_to_obstacle_
            + opt_.k_g()/distance_to_goal_;

    //TODO: consider the minimum excitation speed
    double v = vn_ * exp(-exponent);

    cmd_.speed = getDirSign()*std::max((double)path_driver_->getOptions().min_velocity(), fabs(v));

    ///***///

    ///Direction control

    cmd_.direction_angle = 0;
    double omega = -opt_.k1()*v*orth_proj*std::sin(theta_e)/theta_e - opt_.k2()*std::abs(v)*theta_e;
    omega = boost::algorithm::clamp(omega, -opt_.max_angular_velocity(), opt_.max_angular_velocity());
    cmd_.rotation = omega;
    ROS_INFO("Projection: %f", orth_proj);
    ROS_INFO("v = %f, omega = %f", v, omega);
    ROS_INFO("theta_e = %f", theta_e);
    ///***///


    if(proj_ind_ != path_interpol.n()){
        path_->fireNextWaypointCallback();
    }

    ///plot the moving reference frame together with position vector and error components

    if (visualizer_->MarrayhasSubscriber()) {
        visualizer_->drawFrenetSerretFrame(0, current_pose, xe, ye, path_interpol.p(proj_ind_),
                                           path_interpol.q(proj_ind_), path_interpol.theta_p(proj_ind_));
    }

    ///***///


    if (visualizer_->hasSubscriber()) {
        visualizer_->drawSteeringArrow(1, path_driver_->getRobotPoseMsg(), cmd_.direction_angle, 0.2, 1.0, 0.2);
    }

    ///***///

        *cmd = cmd_;

        return MoveCommandStatus::OKAY;
}

void RobotController_ICR_CCW::publishMoveCommand(const MoveCommand &cmd) const
{
    geometry_msgs::Twist msg;
    msg.linear.x  = cmd.getVelocity();
    msg.linear.y  = 0;
    msg.angular.z = cmd.getRotationalVelocity();

    cmd_pub_.publish(msg);
}

