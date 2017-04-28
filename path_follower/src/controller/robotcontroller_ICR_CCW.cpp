// HEADER
#include <path_follower/controller/robotcontroller_ICR_CCW.h>

// THIRD PARTY
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


// PROJECT
#include <path_follower/pathfollowerparameters.h>
#include <path_follower/utils/cubic_spline_interpolation.h>
#include <path_follower/utils/extended_kalman_filter.h>
#include <cslibs_utils/MathHelper.h>

#include <path_follower/utils/pose_tracker.h>
#include <path_follower/utils/visualizer.h>

// ALGLIB

// SYSTEM
#include <cmath>
#include <deque>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <boost/algorithm/clamp.hpp>

#include <path_follower/factory/controller_factory.h>

REGISTER_ROBOT_CONTROLLER(RobotController_ICR_CCW, ICR_CCW, default_collision_avoider);

using namespace Eigen;


RobotController_ICR_CCW::RobotController_ICR_CCW():
    RobotController(),
    cmd_(this),
    vn_(0),
    Ts_(0.02),
    proj_ind_(0),
    Vl_(0),
    Vr_(0),
    curv_sum_(1e-3),
    distance_to_goal_(1e6),
    distance_to_obstacle_(1e3)
{
    pose_ekf_ = Eigen::Vector3d::Zero();
    ICR_ekf_  = Eigen::Vector3d::Zero();
    last_time_ = ros::Time::now();

    wheel_vel_sub_ = nh_.subscribe<std_msgs::Float64MultiArray>("/wheel_velocities", 10,
                                                                   &RobotController_ICR_CCW::WheelVelocities, this);

    ICR_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("ICR_parameters", 10);

    ekf_points_pub_ = nh_.advertise<visualization_msgs::Marker>("ekf_path", 10);

    path_aug_pub_ = nh_.advertise<visualization_msgs::Marker>("path_aug", 10);

    ekf_path_marker_.header.frame_id = "map";
    ekf_path_marker_.header.stamp = ros::Time();
    ekf_path_marker_.ns = "ekf_path_namespace";
    ekf_path_marker_.id = 51;
    ekf_path_marker_.type = visualization_msgs::Marker::LINE_STRIP;
    ekf_path_marker_.action = visualization_msgs::Marker::ADD;
    ekf_path_marker_.pose.position.x = 0;
    ekf_path_marker_.pose.position.y = 0;
    ekf_path_marker_.pose.position.z = 0;
    ekf_path_marker_.pose.orientation.x = 0.0;
    ekf_path_marker_.pose.orientation.y = 0.0;
    ekf_path_marker_.pose.orientation.z = 0.0;
    ekf_path_marker_.pose.orientation.w = 1.0;
    ekf_path_marker_.scale.x = 0.1;
    ekf_path_marker_.scale.y = 0.0;
    ekf_path_marker_.scale.z = 0.0;
    ekf_path_marker_.color.a = 1.0;
    ekf_path_marker_.color.r = 0.5;
    ekf_path_marker_.color.g = 0.0;
    ekf_path_marker_.color.b = 1.0;


    path_aug_marker_.header.frame_id = "map";
    path_aug_marker_.header.stamp = ros::Time();
    path_aug_marker_.ns = "path_aug_namespace";
    path_aug_marker_.id = 52;
    path_aug_marker_.type = visualization_msgs::Marker::LINE_STRIP;
    path_aug_marker_.action = visualization_msgs::Marker::ADD;
    path_aug_marker_.pose.position.x = 0;
    path_aug_marker_.pose.position.y = 0;
    path_aug_marker_.pose.position.z = 0;
    path_aug_marker_.pose.orientation.x = 0.0;
    path_aug_marker_.pose.orientation.y = 0.0;
    path_aug_marker_.pose.orientation.z = 0.0;
    path_aug_marker_.pose.orientation.w = 1.0;
    path_aug_marker_.scale.x = 0.1;
    path_aug_marker_.scale.y = 0.0;
    path_aug_marker_.scale.z = 0.0;
    path_aug_marker_.color.a = 1.0;
    path_aug_marker_.color.r = 0.0;
    path_aug_marker_.color.g = 0.5;
    path_aug_marker_.color.b = 1.0;

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
    RobotController::initialize();

    //reset the index of the orthogonal projection
    proj_ind_ = 0;

    // desired velocity
    vn_ = std::min(global_opt_->max_velocity(), velocity_);
    ROS_WARN_STREAM("velocity_: " << velocity_ << ", vn: " << vn_);

    //reset the ekf path points
    ekf_path_marker_.points.clear();

    //reset the path_aug points
    path_aug_marker_.points.clear();

    //reset the augmented path
    x_aug_.clear();
    y_aug_.clear();
}


void RobotController_ICR_CCW::WheelVelocities(const std_msgs::Float64MultiArray::ConstPtr& array)
{

    double frw = array->data[0];
    double flw = array->data[1];
    double brw = array->data[2];
    double blw = array->data[3];

    Vl_ = (flw + blw)/2.0;
    Vr_ = (frw + brw)/2.0;

    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_time_).toSec();
    last_time_ = current_time;
    if(Vl_ > 1e-3 || Vr_ > 1e-3){
        ekf_.predict(array, dt);
        pose_ekf_ << ekf_.x_(0), ekf_.x_(1), ekf_.x_(2);
        ICR_ekf_  << ekf_.x_(3), ekf_.x_(4), ekf_.x_(5);
    }

}

void RobotController_ICR_CCW::start()
{

}

void RobotController_ICR_CCW::reset()
{
    RobotController::reset();
}


void RobotController_ICR_CCW::setPath(Path::Ptr path)
{
    RobotController::setPath(path);
}

void RobotController_ICR_CCW::setCurrentPose(const Eigen::Vector3d& pose) {

    ekf_.correct(pose);
    pose_ekf_ << ekf_.x_(0), ekf_.x_(1), ekf_.x_(2);
    ICR_ekf_  << ekf_.x_(3), ekf_.x_(4), ekf_.x_(5);
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
    Eigen::Vector3d current_pose = pose_tracker_->getRobotPose();

    double x_meas = current_pose[0];
    double y_meas = current_pose[1];
    double theta_meas = current_pose[2];
    ///***///

    ///Map the path using current ICR values
    //ICR mapping vector
    double r_x = ICR_ekf_(2);
    double r_y = (ICR_ekf_(1) - ICR_ekf_(0))/2.0;

    //center of the "differential drive"
    x_meas = x_meas + std::cos(theta_meas)*r_x - std::sin(theta_meas)*r_y;
    y_meas = y_meas + std::sin(theta_meas)*r_x + std::cos(theta_meas)*r_y;


    // check for the subpaths, and see if the goal is reached
    if((proj_ind_ == path_interpol.n()-1)) {
        path_->switchToNextSubPath();
        // check if we reached the actual goal or just the end of a subpath
        if (path_->isDone()) {

            cmd_.speed = 0;
            cmd_.direction_angle = 0;
            cmd_.rotation = 0;

            *cmd = cmd_;

            double distance_to_goal_eucl = hypot(x_meas - x_aug_[path_interpol.n()-1],
                                          y_meas - y_aug_[path_interpol.n()-1]);

            ROS_INFO("Final positioning error: %f m", distance_to_goal_eucl);

            return RobotController::MoveCommandStatus::REACHED_GOAL;

        } else {

            ROS_INFO("Next subpath...");
            // interpolate the next subpath
            path_interpol.interpolatePath(path_);
            publishInterpolatedPath();

            // recalculate the driving direction
            //calculateMovingDirection();
        }
    }


    //augmenting the path by the vector r = (x_ICR, (y_ICRl-y_ICRr)/2)
    for(std::size_t i = proj_ind_; i < path_interpol.n(); ++i) {

        //vector of the ICR mapping in world coordinates
        double delta_p =path_interpol.theta_p(i);

        double r_x_m = std::cos(delta_p)*r_x - std::sin(delta_p)*r_y;
        double r_y_m = std::sin(delta_p)*r_x + std::cos(delta_p)*r_y;

        if(i == proj_ind_){
            geometry_msgs::Point pt_aug;
            pt_aug.x = path_interpol.p(i) + r_x_m;
            pt_aug.y = path_interpol.q(i) + r_y_m;
            path_aug_marker_.points.push_back(pt_aug);
        }

        x_aug_.push_back(path_interpol.p(i) + r_x_m);
        y_aug_.push_back(path_interpol.q(i) + r_y_m);
    }

    path_aug_pub_.publish(path_aug_marker_);

    ///***///


    ///calculate the control for the current point on the path
    //robot direction angle in path coordinates
    double theta_e = MathHelper::AngleDelta(path_interpol.theta_p(proj_ind_), theta_meas);

    //robot position vector module
    double r = hypot(x_meas - x_aug_[proj_ind_], y_meas - y_aug_[proj_ind_]);

    //robot position vector angle in world coordinates
    double theta_r = atan2(y_meas - y_aug_[proj_ind_], x_meas - x_aug_[proj_ind_]);

    //robot position vector angle in path coordinates
    double delta_theta = MathHelper::AngleDelta(path_interpol.theta_p(proj_ind_), theta_r);

    //current robot position in path coordinates
    double xe = r * cos(delta_theta);
    double ye = r * sin(delta_theta);

    ///***///



    //find the orthogonal projection to the curve and extract the corresponding index

    double dist = 0;
    double orth_proj = std::numeric_limits<double>::max();

    //this is a hack made for the lemniscate
    int old_ind = proj_ind_;

    for (int i = proj_ind_, n = path_interpol.n(); i < n; i++){

        dist = hypot(x_meas - x_aug_[i], y_meas - y_aug_[i]);
        if((dist < orth_proj) & (i - old_ind >= 0) & (i - old_ind <= 3)){

            orth_proj = dist;
            proj_ind_ = i;

        }

    }

    // distance to the path (path to the right -> positive)
    Eigen::Vector2d path_vehicle(x_meas - x_aug_[proj_ind_], y_meas - y_aug_[proj_ind_]);

    orth_proj = MathHelper::AngleDelta(path_interpol.theta_p(proj_ind_), MathHelper::Angle(path_vehicle)) > 0. ?
                orth_proj : -orth_proj;

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
    double angular_vel = pose_tracker_->getVelocity().angular.z;

    //ensure valid values
    if (distance_to_obstacle_ == 0 || !std::isfinite(distance_to_obstacle_)) distance_to_obstacle_ = 1e-10;
    if (distance_to_goal_ == 0 || !std::isfinite(distance_to_goal_)) distance_to_goal_ = 1e-10;

    double exponent = opt_.k_curv()*fabs(curv_sum_)
            + opt_.k_w()*fabs(angular_vel)
            + opt_.k_o()/distance_to_obstacle_
            + opt_.k_g()/distance_to_goal_;

    //TODO: consider the minimum excitation speed
    double v = vn_ * exp(-exponent);

    cmd_.speed = getDirSign()*std::max((double)global_opt_->min_velocity(), fabs(v));

    ///***///

    ///Direction control

    cmd_.direction_angle = 0;
    double omega = -opt_.k1()*v*orth_proj*std::sin(theta_e)/theta_e - opt_.k2()*std::abs(v)*theta_e;
    omega = boost::algorithm::clamp(omega, -opt_.max_angular_velocity(), opt_.max_angular_velocity());
    cmd_.rotation = omega;
    ///***///


    if(proj_ind_ != path_interpol.n()){
        path_->fireNextWaypointCallback();
    }

    ///plot the moving reference frame together with position vector and error components

    if (visualizer_->MarrayhasSubscriber()) {
        visualizer_->drawFrenetSerretFrame(getFixedFrame(), 0, current_pose, xe, ye, path_interpol.p(proj_ind_),
                                           path_interpol.q(proj_ind_), path_interpol.theta_p(proj_ind_));
    }

    ///***///


    if (visualizer_->hasSubscriber()) {
        visualizer_->drawSteeringArrow(getFixedFrame(), 1, pose_tracker_->getRobotPoseMsg(), cmd_.direction_angle, 0.2, 1.0, 0.2);
    }

    ///***///


    ///Visualize the path estimated by the EKF
    geometry_msgs::Point pt;
    pt.x = pose_ekf_(0);
    pt.y = pose_ekf_(1);
    ekf_path_marker_.points.push_back(pt);

    ekf_points_pub_.publish(ekf_path_marker_);
    ///***///

    //Publish the ICR parameters in a form of an array
    std_msgs::Float64MultiArray ICR_ekf_array;
    ICR_ekf_array.data.resize(3);
    ICR_ekf_array.data[0] = ICR_ekf_(0);
    ICR_ekf_array.data[1] = ICR_ekf_(1);
    ICR_ekf_array.data[2] = ICR_ekf_(2);
    ICR_pub_.publish(ICR_ekf_array);
    //***//

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

