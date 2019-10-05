// HEADER
#include <path_follower/controller/robotcontroller_ackermann_orthexp.h>


// PROJECT
#include <path_follower/parameters/path_follower_parameters.h>
#include <path_follower/utils/cubic_spline_interpolation.h>
#include <path_follower/utils/pose_tracker.h>

#include <path_follower/factory/controller_factory.h>

REGISTER_ROBOT_CONTROLLER(RobotController_Ackermann_OrthogonalExponential, ackermann_orthexp, ackermann);

using namespace Eigen;


RobotController_Ackermann_OrthogonalExponential::RobotController_Ackermann_OrthogonalExponential():
    RobotController_OrthogonalExponential()
{
}

void RobotController_Ackermann_OrthogonalExponential::computeControl()
{
    //control



    // get the pose as pose(0) = x, pose(1) = y, pose(2) = theta
    Eigen::Vector3d current_pose = pose_tracker_->getRobotPose();

    double exp_factor = RobotController::exponentialSpeedControl();
    cmd_.speed = getDirSign() * vn_ * exp_factor;
    // theta_e = theta_path - theta_vehicle
    double theta_e = MathHelper::AngleDelta(current_pose[2], path_interpol.theta_p(proj_ind_));
    if(getDirSign() < 0){
        theta_e = MathHelper::NormalizeAngle(M_PI + theta_e);
    }

    cmd_.direction_angle = atan(-opt_.k()*orth_proj_) + theta_e;

    //***//
}
