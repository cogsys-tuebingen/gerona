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

void RobotController_Differential_OrthogonalExponential::publishMoveCommand(const MoveCommand &cmd) const
{
    geometry_msgs::Twist msg;
    msg.linear.x  = cmd.getVelocity();
    msg.linear.y  = 0;
    msg.angular.z = cmd.getRotationalVelocity();

    cmd_pub_.publish(msg);
}

void RobotController_Differential_OrthogonalExponential::computeControl()
{
    //control

    double exp_factor = RobotController::exponentialSpeedControl();
    cmd_.speed = getDirSign() * vn_ * exp_factor;

    // get the pose as pose(0) = x, pose(1) = y, pose(2) = theta
    Eigen::Vector3d current_pose = pose_tracker_->getRobotPose();

    alpha_e_ = atan(-opt_.k()*orth_proj_);

    // theta_e = theta_path - theta_vehicle
    double theta_e = MathHelper::AngleDelta(current_pose[2], path_interpol.theta_p(proj_ind_));
    if(getDirSign() < 0.0){
        theta_e = MathHelper::NormalizeAngle(M_PI + theta_e);
    }

    cmd_.direction_angle = 0;
    cmd_.rotation = alpha_e_ + theta_e;

    //***//
}

