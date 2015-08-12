/*
 *
 *  Created on: Apr 25, 2015
 *      Author: Lukas Hollaender
 */

#include <path_follower/legacy/robotcontroller_4ws_purepursuit.h>
#include <path_follower/pathfollower.h>
#include <ros/ros.h>

#include "../alglib/interpolation.h"
#include <utils_general/MathHelper.h>

#include <visualization_msgs/Marker.h>

#include <deque>
#include <Eigen/Core>
#include <Eigen/Dense>

RobotController_4WS_PurePursuit::RobotController_4WS_PurePursuit (PathFollower* _path_follower) :
	RobotController_Interpolation(_path_follower),
	waypoint_(0) {

	path_interpol_pub_ = node_handle_.advertise<nav_msgs::Path>("interp_path", 10);


	ROS_INFO("Parameters: factor_lookahead_distance_forward=%f, factor_lookahead_distance_backward=%f"
				"\nvehicle_length=%f\ngoal_tolerance=%f",
				params_.factor_lookahead_distance_forward(), params_.factor_lookahead_distance_backward(),
				params_.vehicle_length(), params_.goal_tolerance());

}

void RobotController_4WS_PurePursuit::reset() {
	waypoint_ = 0;
	RobotController_Interpolation::reset();
}

void RobotController_4WS_PurePursuit::setPath(Path::Ptr path) {
	RobotController_Interpolation::setPath(path);

	Eigen::Vector3d pose = path_driver_->getRobotPose();
	const double theta_diff = MathHelper::AngleDelta(path_interpol.theta_p(0), pose[2]);

	// decide whether to drive forward or backward
	if (theta_diff > M_PI_2 || theta_diff < -M_PI_2)
		setDirSign(-1.f);
	else
		setDirSign(1.f);
}


void RobotController_4WS_PurePursuit::stopMotion() {

	move_cmd_.setVelocity(0.f);
	move_cmd_.setDirection(0.f);

	MoveCommand cmd = move_cmd_;
	publishMoveCommand(cmd);
}

void RobotController_4WS_PurePursuit::start() {
	path_driver_->getCoursePredictor().reset();
}

RobotController::MoveCommandStatus RobotController_4WS_PurePursuit::computeMoveCommand(
		MoveCommand* cmd) {

	if(path_interpol.n() <= 2)
		return RobotController::MoveCommandStatus::ERROR;

	ROS_INFO("======================");

	Eigen::Vector3d pose = path_driver_->getRobotPose();

	if (reachedGoal(pose)) {
		path_->switchToNextSubPath();
		if (path_->isDone()) {
			move_cmd_.setDirection(0.);
			move_cmd_.setVelocity(0.);

			*cmd = move_cmd_;

			return RobotController::MoveCommandStatus::REACHED_GOAL;

		} else {

			ROS_INFO("Next subpath...");

			try {
				path_interpol.interpolatePath(path_);
				// publishInterpolatedPath();

			} catch(const alglib::ap_error& error) {
				throw std::runtime_error(error.msg);
			}

			waypoint_ = 0;
			setDirSign(-getDirSign());
		}
	}

	double lookahead_distance = velocity_;
	if(getDirSign() >= 0.)
		lookahead_distance *= params_.factor_lookahead_distance_forward();
	else
		lookahead_distance *= params_.factor_lookahead_distance_backward();

	// angle between vehicle theta and the connection between the rear axis and the look ahead point
	const double alpha = computeAlpha(lookahead_distance, pose);

	const double delta = asin(params_.vehicle_length() * sin(alpha) / lookahead_distance);

	move_cmd_.setDirection((float) delta);
	move_cmd_.setVelocity(getDirSign() * (float) velocity_);

	ROS_INFO("Command: vel=%f, angle=%f", velocity_, delta);

	*cmd = move_cmd_;

	return RobotController::MoveCommandStatus::OKAY;
}

void RobotController_4WS_PurePursuit::publishMoveCommand(
		const MoveCommand& cmd) const {

	geometry_msgs::Twist msg;
	msg.linear.x  = cmd.getVelocity();
	msg.linear.y  = 0;
	msg.angular.z = cmd.getDirectionAngle();

	cmd_pub_.publish(msg);
}

double RobotController_4WS_PurePursuit::computeAlpha(double& lookahead_distance,
																			  const Eigen::Vector3d& pose) {

	// TODO: correct angle, when the goal is near

	double distance, dx, dy;
	for (unsigned int i = waypoint_; i < path_interpol.n(); ++i) {
		dx = path_interpol.p(i) - pose[0];
		dy = path_interpol.q(i) - pose[1];

		distance = hypot(dx, dy);
		waypoint_ = i;
		if (distance >= lookahead_distance)
			break;
	}

	// angle between the connection line and the vehicle orientation
	double alpha = MathHelper::AngleDelta(pose[2], atan2(dy, dx));

	if (getDirSign() < 0.) {
		if (alpha > 0.)
			alpha = M_PI - alpha;
		else
			alpha = -M_PI - alpha;
	}
	// set lookahead_distance to the actual distance
	lookahead_distance = distance;

	// line to lookahead point
	geometry_msgs::Point from, to;
	from.x = pose[0]; from.y = pose[1];
	to.x = path_interpol.p(waypoint_); to.y = path_interpol.q(waypoint_);
	visualizer_->drawLine(12341234, from, to, "map", "geo", 1, 0, 0, 1, 0.01);

#ifdef DEBUG
	ROS_INFO("LookAheadPoint: index=%i, x=%f, y=%f", waypoint_, path_interpol.p(waypoint_)
				, path_interpol.q(waypoint_));
	ROS_INFO("Pose: x=%f, y=%f, theta=%f", pose[0], pose[1], pose[2]);
	ROS_INFO("Alpha=%f, dir_sign=%f", alpha, dir_sign_);
#endif

	return alpha;
}
