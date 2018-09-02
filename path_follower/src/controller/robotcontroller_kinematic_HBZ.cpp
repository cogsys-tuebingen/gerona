// HEADER
#include <path_follower/controller/robotcontroller_kinematic_HBZ.h>

// THIRD PARTY
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


// PROJECT
#include <path_follower/parameters/path_follower_parameters.h>
#include <path_follower/utils/cubic_spline_interpolation.h>
#include <cslibs_utils/MathHelper.h>
#include <path_follower/utils/pose_tracker.h>
#include <path_follower/utils/visualizer.h>
#include <path_follower/utils/obstacle_cloud.h>
#include <path_follower/collision_avoidance/collision_avoider.h>
#include <path_follower/factory/controller_factory.h>


// SYSTEM
#include <cmath>
#include <deque>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <boost/algorithm/clamp.hpp>
#include <pcl_ros/point_cloud.h>
#include <path_follower/factory/controller_factory.h>

REGISTER_ROBOT_CONTROLLER(RobotController_Kinematic_HBZ, kinematic_hbz, default_collision_avoider);

using namespace Eigen;


RobotController_Kinematic_HBZ::RobotController_Kinematic_HBZ():
    RobotController(),
    cmd_(this),
    vn_(0),
    Ts_(0.02),
    ind_(0),
    proj_ind_(0),
    xe_(0),
    ye_(0),
    theta_e_(0),
    x_meas_(0),
    y_meas_(0),
    theta_meas_(0),
    angular_vel_(0),
    Vl_(0),
    Vr_(0),
    delta_(0)
{

    wheel_velocities_ = nh_.subscribe<std_msgs::Float64MultiArray>("wheel_velocities", 10,
                                                                   &RobotController_Kinematic_HBZ::WheelVelocities, this);

}

void RobotController_Kinematic_HBZ::stopMotion()
{

    cmd_.speed = 0;
    cmd_.direction_angle = 0;
    cmd_.rotation = 0;

    MoveCommand mcmd = cmd_;
    publishMoveCommand(mcmd);
}

void RobotController_Kinematic_HBZ::initialize()
{
    RobotController::initialize();

    //reset the index of the current point on the path
    ind_ = 0;

    theta_e_ = 0.0;

    // desired velocity
    vn_ = std::min(PathFollowerParameters::getInstance()->max_velocity(), velocity_);
    ROS_DEBUG_STREAM("velocity_: " << velocity_ << ", vn: " << vn_);


}

void RobotController_Kinematic_HBZ::WheelVelocities(const std_msgs::Float64MultiArray::ConstPtr& array)
{
    double frw = array->data[0];
    double flw = array->data[1];
    double brw = array->data[2];
    double blw = array->data[3];

    Vl_ = (flw + blw)/2.0;
    Vr_ = (frw + brw)/2.0;
}


void RobotController_Kinematic_HBZ::start()
{

}

void RobotController_Kinematic_HBZ::reset()
{
    RobotController::reset();
}

void RobotController_Kinematic_HBZ::setPath(Path::Ptr path)
{
    RobotController::setPath(path);
}

double RobotController_Kinematic_HBZ::computeSpeed()
{
    double v = vn_;

    double V1 = 1.0/2.0*(std::pow(xe_,2) + std::pow(ye_,2) + 1.0/opt_.lambda()*fabs(sin(theta_e_-delta_)));

    if(angular_vel_ > 0){

        if(V1 >= opt_.epsilon()){
            v = (-opt_.alpha_r()*opt_.y_ICR_l()*vn_)/(opt_.y_ICR_r() - opt_.y_ICR_l());
        }
        else if(V1 < opt_.epsilon()){
            v = (opt_.alpha_r()*vn_)/(1 + std::fabs(opt_.y_ICR_r()*path_interpol.curvature(ind_)));
        }
    }
    else if(angular_vel_ <= 0){

        if(V1 >= opt_.epsilon()){
            v = (opt_.alpha_l()*opt_.y_ICR_r()*vn_)/(opt_.y_ICR_r() - opt_.y_ICR_l());
        }
        else if(V1 < opt_.epsilon()){
            v = (opt_.alpha_l()*vn_)/(1 + std::fabs(opt_.y_ICR_l()*path_interpol.curvature(ind_)));
        }
    }

    return v;
}

RobotController::MoveCommandStatus RobotController_Kinematic_HBZ::computeMoveCommand(MoveCommand *cmd)
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

    const geometry_msgs::Twist v_meas_twist = pose_tracker_->getVelocity();

    double v_meas = getDirSign() * sqrt(v_meas_twist.linear.x * v_meas_twist.linear.x
            + v_meas_twist.linear.y * v_meas_twist.linear.y);

    x_meas_ = current_pose[0];
    y_meas_ = current_pose[1];
    theta_meas_ = current_pose[2];
    ///***///

    RobotController::findOrthogonalProjection();

    if(RobotController::isGoalReached(cmd)){
       return RobotController::MoveCommandStatus::REACHED_GOAL;
    }

    ///compute the control for the current point on the path

    //robot direction angle in path coordinates
    theta_e_ = MathHelper::AngleDelta(path_interpol.theta_p(ind_), theta_meas_);

    //robot position vector module
    double r = hypot(x_meas_ - path_interpol.p(ind_), y_meas_ - path_interpol.q(ind_));

    //robot position vector angle in world coordinates
    double theta_r = atan2(y_meas_ - path_interpol.q(ind_), x_meas_ - path_interpol.p(ind_));

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


    ///Lyapunov-curvature speed control - skid steering

       double v = computeSpeed();

    ///***///


    ///compute the next point on the path

    double s_old = path_interpol.s_new();

    //calculate the speed of the "virtual vehicle"
    double s_prim_tmp = v * cos(theta_e_) + opt_.x_ICR()*angular_vel_*sin(theta_e_) + opt_.k1() * xe_;
    path_interpol.set_s_prim(s_prim_tmp > 0 ? s_prim_tmp : 0);

    //approximate the first derivative and calculate the next point
    double s_temp = Ts_*path_interpol.s_prim() + s_old;
    path_interpol.set_s_new(s_temp > 0 ? s_temp : 0);

    ///***///

    double exp_factor = RobotController::exponentialSpeedControl();
    v = v * exp_factor;

    //cmd_.speed = getDirSign()*std::max((double)PathFollowerParameters::getInstance()->min_velocity(), fabs(v));
    //allow the speed to be reduced to zero
    cmd_.speed = getDirSign() * v;

    ///***///


    ///Compute the delta_ and its derivative

    double delta_old = delta_;

    delta_ = MathHelper::AngleClamp(-getDirSign()*opt_.theta_a()*tanh(ye_));

    double delta_prim = (delta_ - delta_old)/Ts_;
    ///***///


    ///Direction control

    cmd_.direction_angle = 0;

    double theta_diff = MathHelper::AngleDelta(delta_, theta_e_);

    double trig_ratio = (fabs(sin(theta_diff)))/(sin(theta_diff)*cos(theta_diff));
    double omega = delta_prim
            - opt_.lambda()*trig_ratio*ye_*v*sin(theta_e_)
            + opt_.lambda()*trig_ratio*ye_*opt_.x_ICR()*angular_vel_*cos(theta_e_)
            - opt_.k2()*trig_ratio*(theta_diff)*(theta_diff);

    omega += path_interpol.curvature(ind_)*path_interpol.s_prim();
    cmd_.rotation = boost::algorithm::clamp(omega, -opt_.max_angular_velocity(), opt_.max_angular_velocity());

    ///***///


    ///Plot the moving reference frame together with position vector and error components

    if (visualizer_->MarrayhasSubscriber()) {
        visualizer_->drawFrenetSerretFrame(getFixedFrame(), 0, current_pose, xe_, ye_, path_interpol.p(ind_),
                                           path_interpol.q(ind_), path_interpol.theta_p(ind_));
    }

    ///***///


    ///Compute the index of the new point

    double s_diff = std::numeric_limits<double>::max();
    uint old_ind = ind_;
    //if the robot is not moving, the Frenet-Serret frame should also not move
    if(fabs(v_meas) < 1e-3){
        s_diff = 0.0;
    }

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
        visualizer_->drawSteeringArrow(pose_tracker_->getFixedFrameId(), 1, pose_tracker_->getRobotPoseMsg(), cmd_.direction_angle, 0.2, 1.0, 0.2);
    }

    ///***///

    *cmd = cmd_;

    return MoveCommandStatus::OKAY;
}

void RobotController_Kinematic_HBZ::publishMoveCommand(const MoveCommand &cmd) const
{
    geometry_msgs::Twist msg;
    msg.linear.x  = cmd.getVelocity();
    msg.linear.y  = 0;
    msg.angular.z = cmd.getRotationalVelocity();

    cmd_pub_.publish(msg);
}

