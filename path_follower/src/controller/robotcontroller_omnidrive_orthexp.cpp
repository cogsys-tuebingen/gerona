// PROJECT
#include <path_follower/parameters/path_follower_parameters.h>
#include <path_follower/controller/robotcontroller_omnidrive_orthexp.h>
#include <path_follower/utils/pose_tracker.h>
#include <cslibs_utils/MathHelper.h>

// SYSTEM
#include <boost/algorithm/clamp.hpp>

#include <path_follower/factory/controller_factory.h>

REGISTER_ROBOT_CONTROLLER(RobotController_Omnidrive_OrthogonalExponential, omnidrive_orthexp, omnidrive);

using namespace Eigen;

namespace {
//! Module name, that is used for ros console output
const std::string MODULE = "controller";
}

RobotController_Omnidrive_OrthogonalExponential::RobotController_Omnidrive_OrthogonalExponential():
    RobotController_OrthogonalExponential(),
    e_theta_curr_(0)
{

}

void RobotController_Omnidrive_OrthogonalExponential::computeControl()
{
    // get the pose as pose(0) = x, pose(1) = y, pose(2) = theta
    Eigen::Vector3d current_pose = pose_tracker_->getRobotPose();

    double e_theta_new = MathHelper::NormalizeAngle(theta_des_ - current_pose[2]);
    double e_theta_prim = (e_theta_new - e_theta_curr_)/Ts_;

    e_theta_curr_ = e_theta_new;

    double exp_factor = RobotController::exponentialSpeedControl();
    cmd_.speed = vn_* exp_factor;
    cmd_.direction_angle = atan(-opt_.k()*orth_proj_) + path_interpol.theta_p(proj_ind_) - current_pose[2];
    double omega = opt_.kp()*e_theta_curr_ + opt_.kd()*e_theta_prim;
    cmd_.rotation = boost::algorithm::clamp(omega, -opt_.max_angular_velocity(), opt_.max_angular_velocity());
}
