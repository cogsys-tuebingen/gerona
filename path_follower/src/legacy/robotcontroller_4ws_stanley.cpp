#include <path_follower/legacy/robotcontroller_4ws_stanley.h>

#include <path_follower/pathfollower.h>
#include <ros/ros.h>

#include "../alglib/interpolation.h"
#include <utils_general/MathHelper.h>

#include <visualization_msgs/Marker.h>

#include <deque>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <limits>
#include <boost/algorithm/clamp.hpp>

RobotController_4WS_Stanley::RobotController_4WS_Stanley(PathFollower* _path_follower) :
	RobotController_Interpolation(_path_follower) {

	ROS_INFO("Parameters: k_forward=%f, k_backward=%f\n"
				"vehicle_length=%f\n"
				"goal_tolerance=%f",
				params_.k_forward(), params_.k_backward(),
				params_.vehicle_length(),
				params_.goal_tolerance());

}

void RobotController_4WS_Stanley::stopMotion() {

	move_cmd_.setVelocity(0.f);
	move_cmd_.setDirection(0.f);

	MoveCommand cmd = move_cmd_;
	publishMoveCommand(cmd);
}

void RobotController_4WS_Stanley::start() {
	path_driver_->getCoursePredictor().reset();
}

void RobotController_4WS_Stanley::setPath(Path::Ptr path) {
	RobotController_Interpolation::setPath(path);

	Eigen::Vector3d pose = path_driver_->getRobotPose();
	const double theta_diff = MathHelper::AngleDelta(path_interpol.theta_p(0), pose[2]);

	// decide whether to drive forward or backward
	if (theta_diff > M_PI_2 || theta_diff < -M_PI_2)
		setDirSign(-1.f);
	else
		setDirSign(1.f);
}

RobotController::MoveCommandStatus RobotController_4WS_Stanley::computeMoveCommand(
		MoveCommand* cmd) {

	ROS_INFO("===============================");

	if(path_interpol.n() <= 2)
		return RobotController::MoveCommandStatus::ERROR;

	const Eigen::Vector3d pose = path_driver_->getRobotPose();
	const geometry_msgs::Twist velocity_measured = path_driver_->getVelocity();

	// goal test
	if (reachedGoal(pose)) {
		path_->switchToNextSubPath();
		// check if we reached the actual goal or just the end of a subpath
		if (path_->isDone()) {
			move_cmd_.setDirection(0.);
			move_cmd_.setVelocity(0.);

			*cmd = move_cmd_;

			return RobotController::MoveCommandStatus::REACHED_GOAL;

		} else {

			ROS_INFO("Next subpath...");
			// interpolate the next subpath
			try {
				path_interpol.interpolatePath(path_);
			} catch(const alglib::ap_error& error) {
				throw std::runtime_error(error.msg);
			}
			setDirSign(-getDirSign());
		}
	}

	// compute the length of the orthogonal projection and the according path index
	double min_dist = std::numeric_limits<double>::max();
	unsigned int ind = 0;
	for (unsigned int i = 0; i < path_interpol.n(); ++i) {
		const double dx = path_interpol.p(i) - pose[0];
		const double dy = path_interpol.q(i) - pose[1];

		const double dist = hypot(dx, dy);
		if (dist < min_dist) {
			min_dist = dist;
			ind = i;
		}
	}

	// draw a line to the orthogonal projection
	geometry_msgs::Point from, to;
	from.x = pose[0]; from.y = pose[1];
	to.x = path_interpol.p(ind); to.y = path_interpol.q(ind);
	visualizer_->drawLine(12341234, from, to, "map", "kinematic", 1, 0, 0, 1, 0.01);


	// distance to the path (path to the right -> positive)
	Eigen::Vector2d path_vehicle(pose[0] - path_interpol.p(ind), pose[1] - path_interpol.q(ind));

	double d =
			MathHelper::AngleDelta(MathHelper::Angle(path_vehicle), path_interpol.theta_p(ind)) < 0. ?
				-min_dist : min_dist;


	// theta_e = theta_vehicle - theta_path (orientation error)
	double theta_e = MathHelper::AngleDelta(pose[2], path_interpol.theta_p(ind));

	// if we drive backwards invert d and set theta_e to the complementary angle
	if (getDirSign() < 0.) {
		d = -d;
		theta_e = theta_e > 0.? M_PI - theta_e : -M_PI - theta_e;
	}

	const double k = getDirSign() > 0. ? params_.k_forward() : params_.k_backward();
	const double v = max(abs(velocity_measured.linear.x), 0.2);

	const double tan_theta_e = tan(theta_e);
	const double kd_v = k * d / v;

	const double phi = asin((kd_v + tan_theta_e) / (2. + 2. * kd_v * tan_theta_e));

	if (phi == NAN) {
		ROS_ERROR("Got NAN phi");
		return RobotController::MoveCommandStatus::ERROR;
	}

	ROS_INFO("d=%f, theta_e=%f\ndir=%f, v=%f, kd_v=%f, phi=%f",
				d, theta_e, getDirSign(), v, kd_v, phi);

	move_cmd_.setDirection((float) phi);
	move_cmd_.setVelocity(getDirSign() * (float) velocity_);

	*cmd = move_cmd_;

	return RobotController::MoveCommandStatus::OKAY;
}

void RobotController_4WS_Stanley::publishMoveCommand(
		const MoveCommand& cmd) const {

	geometry_msgs::Twist msg;
	msg.linear.x  = cmd.getVelocity();
	msg.linear.y  = 0;
	msg.angular.z = cmd.getDirectionAngle();

	cmd_pub_.publish(msg);
}
