#include <path_follower/controller/robotcontroller_ackermann_stanley.h>



#include <path_follower/utils/pose_tracker.h>
#include <ros/ros.h>
#include <path_follower/utils/visualizer.h>

#include <cslibs_utils/MathHelper.h>

#include <visualization_msgs/Marker.h>

#include <deque>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <limits>
#include <boost/algorithm/clamp.hpp>

#include <path_follower/factory/controller_factory.h>

REGISTER_ROBOT_CONTROLLER(RobotController_Ackermann_Stanley, ackermann_stanley);

RobotController_Ackermann_Stanley::RobotController_Ackermann_Stanley():
    RobotController()
{

	ROS_INFO("Parameters: k_forward=%f, k_backward=%f\n"
				"vehicle_length=%f\n"
				"factor_steering_angle=%f\n"
				"goal_tolerance=%f",
				params_.k_forward(), params_.k_backward(),
				params_.vehicle_length(),
				params_.factor_steering_angle(),
				params_.goal_tolerance());

}

void RobotController_Ackermann_Stanley::stopMotion() {

	move_cmd_.setVelocity(0.f);
	move_cmd_.setDirection(0.f);

	MoveCommand cmd = move_cmd_;
	publishMoveCommand(cmd);
}

void RobotController_Ackermann_Stanley::start() {

}

void RobotController_Ackermann_Stanley::reset() {

    RobotController::reset();
}

void RobotController_Ackermann_Stanley::setPath(Path::Ptr path) {
    RobotController::setPath(path);
}

RobotController::MoveCommandStatus RobotController_Ackermann_Stanley::computeMoveCommand(
		MoveCommand* cmd) {

	if(path_interpol.n() <= 2)
		return RobotController::MoveCommandStatus::ERROR;

    const Eigen::Vector3d pose = pose_tracker_->getRobotPose();

    double d = -RobotController::findOrthogonalProjection();

    if(RobotController::isGoalReached(cmd)){
       return RobotController::MoveCommandStatus::REACHED_GOAL;
    }

	// draw a line to the orthogonal projection
	geometry_msgs::Point from, to;
	from.x = pose[0]; from.y = pose[1];
    to.x = path_interpol.p(proj_ind_); to.y = path_interpol.q(proj_ind_);
    visualizer_->drawLine(12341234, from, to, getFixedFrame(), "kinematic", 1, 0, 0, 1, 0.01);

	// theta_e = theta_vehicle - theta_path (orientation error)
    double theta_e = MathHelper::AngleDelta(pose[2], path_interpol.theta_p(proj_ind_));

    // if we drive backwards invert d and set theta_e to the complementary angle
    if (getDirSign() < 0.) {
        d = -d;
        theta_e = theta_e > 0.? M_PI - theta_e : -M_PI - theta_e;
    }

	const double k = getDirSign() > 0. ? params_.k_forward() : params_.k_backward();

	const double phi = theta_e + atan2(k * d, velocity_);

	// This is the accurate steering angle for 4 wheel steering
	const float phi_actual = (float) asin(params_.factor_steering_angle() * sin(phi));

	move_cmd_.setDirection(phi_actual);
	move_cmd_.setVelocity(getDirSign() * (float) velocity_);

	*cmd = move_cmd_;

	return RobotController::MoveCommandStatus::OKAY;
}

void RobotController_Ackermann_Stanley::publishMoveCommand(
		const MoveCommand& cmd) const {

	geometry_msgs::Twist msg;
	msg.linear.x  = cmd.getVelocity();
	msg.linear.y  = 0;
	msg.angular.z = cmd.getDirectionAngle();

	cmd_pub_.publish(msg);
}
