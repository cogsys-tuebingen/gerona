// HEADER
#include <path_follower/controller/robotcontroller_differential_orthexp.h>

// PROJECT
#include <path_follower/parameters/path_follower_parameters.h>
#include <path_follower/utils/cubic_spline_interpolation.h>
#include <path_follower/utils/pose_tracker.h>

#include <path_follower/factory/controller_factory.h>

REGISTER_ROBOT_CONTROLLER(RobotController_Differential_OrthogonalExponential, differential_orthexp, differential);

using namespace Eigen;


RobotController_Differential_OrthogonalExponential::RobotController_Differential_OrthogonalExponential():
    RobotController_OrthogonalExponential(),
    alpha_e_(0.0)
{

}

void RobotController_Differential_OrthogonalExponential::computeControl()
{
    //control

    double exp_factor = RobotController::exponentialSpeedControl();
    ROS_INFO("vn_: %f", vn_);
    cmd_.speed = vn_ * exp_factor;

    // get the pose as pose(0) = x, pose(1) = y, pose(2) = theta
    Eigen::Vector3d current_pose = pose_tracker_->getRobotPose();

    double last_alpha_e = alpha_e_;
    alpha_e_ = atan(-opt_.k()*orth_proj_);
    cmd_.direction_angle = alpha_e_ + path_interpol.theta_p(proj_ind_) - current_pose[2];

    cmd_.rotation = (alpha_e_ - last_alpha_e) / Ts_ + cmd_.direction_angle;

    //***//
}

