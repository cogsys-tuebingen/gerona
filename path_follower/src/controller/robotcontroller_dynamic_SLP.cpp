// HEADER
#include <path_follower/controller/robotcontroller_dynamic_SLP.h>

// THIRD PARTY
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// PROJECT
#include <path_follower/pathfollowerparameters.h>
#include <path_follower/utils/cubic_spline_interpolation.h>
#include <cslibs_utils/MathHelper.h>
#include <path_follower/utils/pose_tracker.h>
#include <path_follower/utils/visualizer.h>

// SYSTEM
#include <cmath>
#include <deque>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <boost/algorithm/clamp.hpp>

#include <path_follower/factory/controller_factory.h>

REGISTER_ROBOT_CONTROLLER(RobotController_Dynamic_SLP, dynamic_SLP);

using namespace Eigen;


RobotController_Dynamic_SLP::RobotController_Dynamic_SLP():
    RobotController(),
    cmd_(this),
    vn_(0),
    Ts_(0.02),
    ind_(0),
    proj_ind_(0),
    xe_(0),
    ye_(0),
    theta_e_(0),
    delta_(0),
    s_prim_(0),
    vx_(0),
    zeta_(0),
    epsilon_(0),
    curv_sum_(1e-3),
    distance_to_goal_(1e6),
    distance_to_obstacle_(1e3)
{
}

void RobotController_Dynamic_SLP::stopMotion()
{
    cmd_.tau_fl = 0;
    cmd_.tau_fr = 0;
    cmd_.tau_br = 0;
    cmd_.tau_bl = 0;

    MoveCommand mcmd = cmd_;
    publishMoveCommand(mcmd);
}

void RobotController_Dynamic_SLP::initialize()
{
    RobotController::initialize();

    //reset the index of the current point on the path
    ind_ = 0;

    // desired velocity
    vn_ = std::min(global_opt_->max_velocity(), velocity_);
    ROS_WARN_STREAM("velocity_: " << velocity_ << ", vn: " << vn_);

    //calculate the maximal allowed torque (this should be defined as constant later)
    max_torque_ = opt_.gearbox()*opt_.Kt()*opt_.max_current();


}

void RobotController_Dynamic_SLP::start()
{

}

void RobotController_Dynamic_SLP::reset()
{
    RobotController::reset();
}


void RobotController_Dynamic_SLP::setPath(Path::Ptr path)
{
    RobotController::setPath(path);
}

RobotController::MoveCommandStatus RobotController_Dynamic_SLP::computeMoveCommand(MoveCommand *cmd)
{
    // omni drive can rotate.
    *cmd = MoveCommand(true, true);


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

    RobotController::findOrthogonalProjection();

    if(RobotController::isGoalReached(cmd)){
       return RobotController::MoveCommandStatus::REACHED_GOAL;
    }

    ///calculate the error coordinates for the current point on the path

    //robot direction angle in path coordinates
    double theta_e_new = MathHelper::AngleDelta(path_interpol.theta_p(ind_), theta_meas);
    double theta_e_prim = (theta_e_new - theta_e_)/Ts_;
    theta_e_ = theta_e_new;

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
        theta_e_ = MathHelper::NormalizeAngle(M_PI + theta_e_);
        ROS_WARN_THROTTLE(1, "Driving backwards...");
    }

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
    distance_to_goal_ = path_interpol.s(path_interpol.n()-1) - path_interpol.s(proj_ind_);

    //get the robot's current angular velocity
    //double angular_vel = pose_tracker_->getVelocity().angular.z;
    ///***///


    ///Calculate the delta_ and its derivative

    double delta_old = delta_;

    delta_ = MathHelper::AngleClamp(-getDirSign()*opt_.theta_a()*tanh(ye_));

    double delta_prim = (delta_ - delta_old)/Ts_;
    ///***///


    ///Lyapunov-curvature speed control

//    //Lyapunov function as a measure of the path following error
//    double v = vn_;
//    double V1 = 0.5*(std::pow(xe_,2) + std::pow(ye_,2)) + (0.5/opt_.gamma())*std::pow((theta_e - delta_),2);

//    //use v/2 as the minimum speed, and allow larger values when the error is small
//    if (V1 >= opt_.epsilon()) v = 0.5*v;

//    else if (V1 < opt_.epsilon()) v = v/(1 + opt_.b()*std::abs(path_interpol.curvature(ind_)));

    ///***///


    ///Calculate the next point on the path

    double s_old = path_interpol.s_new();

    double s_prim_old = s_prim_;

    //calculate the speed of the "virtual vehicle"
    double s_prim_ = vn_ * cos(theta_e_) + opt_.k1() * xe_;
    path_interpol.set_s_prim(s_prim_ > 0 ? s_prim_ : 0);

    double s_sek = (s_prim_ - s_prim_old)/Ts_;

    //approximate the first derivative and calculate the next point
    double s_temp = Ts_*path_interpol.s_prim() + s_old;
    path_interpol.set_s_new(s_temp > 0 ? s_temp : 0);

    ///***///


    ///Calculate the control


//    //omega_m = theta_e_prim + curv*s_prim
//    double omega_m = delta_prim - opt_.gamma()*ye_*v*(sin(theta_e) - sin(delta_))
//                    /(theta_e - delta_) - opt_.k2()*(theta_e - delta_) + path_interpol.curvature(ind_)*path_interpol.s_prim();


//    omega_m = boost::algorithm::clamp(omega_m, -opt_.max_angular_velocity(), opt_.max_angular_velocity());
//    //cmd_.rotation = omega_m;

    double u1 = 0.0;
    double u2 = 0.0;

    double c1 = opt_.I()*opt_.r()/opt_.w();
    double c2 = opt_.m()*opt_.r();

    vx_ = pose_tracker_->getVelocity().linear.x;

    double zeta_old = zeta_;
    zeta_ = delta_prim - opt_.gamma()*(sin(theta_e_) - sin(delta_))/(theta_e_ - delta_) - opt_.k2()*(theta_e_ - delta_);
    double zeta_prim = (zeta_ - zeta_old)/Ts_;

    double epsilon_old = epsilon_;
    epsilon_ = theta_e_prim - zeta_;
    double epsilon_prim = (epsilon_ - epsilon_old)/Ts_;

    double vx_prim = 0;
    double omega_m_prim = 0;

    omega_m_prim = epsilon_prim + zeta_prim + path_interpol.curvature(ind_)*s_sek
            + path_interpol.curvature_prim(ind_)*s_prim_*s_prim_;

    vx_prim = -opt_.k4()*(vx_ - vn_);

    u1 = c1*omega_m_prim;
    u2 = c2*vx_prim;

    //right tread
    double tau1 = (u1 + u2)/2.0;
    //left tread
    double tau2 = (u2 - u1)/2.0;

    cmd_.tau_fl = boost::algorithm::clamp(tau2/2.0, -max_torque_, max_torque_);
    cmd_.tau_bl = boost::algorithm::clamp(tau2/2.0, -max_torque_, max_torque_);
    cmd_.tau_fr = boost::algorithm::clamp(tau1/2.0, -max_torque_, max_torque_);
    cmd_.tau_br = boost::algorithm::clamp(tau1/2.0, -max_torque_, max_torque_);


    ///***///


    ///Exponential speed control

//    //ensure valid values
//    if (distance_to_obstacle_ == 0 || !std::isfinite(distance_to_obstacle_)) distance_to_obstacle_ = 1e-10;
//    if (distance_to_goal_ == 0 || !std::isfinite(distance_to_goal_)) distance_to_goal_ = 1e-10;

//    double exponent = opt_.k_curv()*fabs(curv_sum_)
//            + opt_.k_w()*fabs(angular_vel)
//            + opt_.k_o()/distance_to_obstacle_
//            + opt_.k_g()/distance_to_goal_;

//    //TODO: consider the minimum excitation speed
//    v = v * exp(-exponent);

//    //cmd_.speed = getDirSign()*std::max((double)opt_.min_velocity(), fabs(v));

    ///***///



    ///plot the moving reference frame together with position vector and error components

    if (visualizer_->hasSubscriber()) {
        visualizer_->drawFrenetSerretFrame(getFixedFrame(), 0, current_pose, xe_, ye_, path_interpol.p(ind_),
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
      //  visualizer_->drawSteeringArrow(pose_tracker_->getFixedFrameId(), 1, pose_tracker_->getRobotPoseMsg(), cmd_.direction_angle, 0.2, 1.0, 0.2);
    }

    ///***///

        *cmd = cmd_;

        return MoveCommandStatus::OKAY;
}

void RobotController_Dynamic_SLP::publishMoveCommand(const MoveCommand &cmd) const
{   
    std_msgs::Float64MultiArray torques;
    torques.data.clear();

    torques.data.push_back(cmd.getWheelTorqueFL());
    torques.data.push_back(cmd.getWheelTorqueFR());
    torques.data.push_back(cmd.getWheelTorqueBR());
    torques.data.push_back(cmd.getWheelTorqueBL());

    cmd_pub_.publish(torques);
}
