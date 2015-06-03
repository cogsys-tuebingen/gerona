/*
 * robotcontrollerackermanngeometrical.cpp
 *
 *  Created on: Apr 25, 2015
 *      Author: Lukas Hollaender
 */

#include <path_follower/legacy/robotcontroller_ackermann_purepursuit.h>
#include <path_follower/pathfollower.h>
#include <ros/ros.h>

#include "../alglib/interpolation.h"
#include <utils_general/MathHelper.h>

#include <visualization_msgs/Marker.h>

#include <deque>
#include <Eigen/Core>
#include <Eigen/Dense>

#define DEBUG


Robotcontroller_Ackermann_PurePursuit::Robotcontroller_Ackermann_PurePursuit(
		PathFollower* _path_follower) :
	RobotController(_path_follower) {

	initialized = false;

	visualizer = Visualizer::getInstance();

	path_marker.ns = "pure_pursuit";
	path_marker.header.frame_id = "/map";
	path_marker.header.stamp = ros::Time();
	path_marker.action = visualization_msgs::Marker::ADD;
	path_marker.id = 12341235;
	path_marker.color.r = 0;
	path_marker.color.g = 0;
	path_marker.color.b = 1;
	path_marker.color.a = 1.0;
	path_marker.scale.x = 0.03;
	path_marker.scale.y = 0.03;
	path_marker.scale.z = 0.03;
	path_marker.type = visualization_msgs::Marker::POINTS;

	path_interpol_pub = node_handle.advertise<nav_msgs::Path>("interp_path", 10);


	ROS_INFO("Parameters: factor_lookahead_distance=%f\n"
				"vehicle_length=%f", params.factor_lookahead_distance(), params.vehicle_length());

}

Robotcontroller_Ackermann_PurePursuit::~Robotcontroller_Ackermann_PurePursuit() {
}

void Robotcontroller_Ackermann_PurePursuit::stopMotion() {

	move_cmd.setVelocity(0.f);
	move_cmd.setDirection(0.f);

	MoveCommand cmd = move_cmd;
	publishMoveCommand(cmd);
}

void Robotcontroller_Ackermann_PurePursuit::start() {
	path_driver_->getCoursePredictor().reset();
}

void Robotcontroller_Ackermann_PurePursuit::reset() {
	initialized = false;
}

void Robotcontroller_Ackermann_PurePursuit::setPath(Path::Ptr path) {

	RobotController::setPath(path);

	if (initialized) {
		return;
	}

	ROS_INFO("Interpolating new path");

	try {
		path_interpol.interpolatePath(path_);
		publishInterpolatedPath();

	} catch (const alglib::ap_error& error) {
		throw std::runtime_error(error.msg);
	}

	initialize();
}

RobotController::MoveCommandStatus Robotcontroller_Ackermann_PurePursuit::computeMoveCommand(
		MoveCommand* cmd) {

	if(path_interpol.length() <= 2)
		return RobotController::MoveCommandStatus::ERROR;

	Eigen::Vector3d pose = path_driver_->getRobotPose();

	if (reachedGoal(pose)) {
		move_cmd.setDirection(0.);
		move_cmd.setVelocity(0.);

		*cmd = move_cmd;

#ifdef DEBUG
		ROS_INFO("Reached goal.");
#endif

		return RobotController::MoveCommandStatus::REACHED_GOAL;
	}

	/*
	 * IDEAS:
	 * - lookahead_distance depending on the curvature
	 * - remember current waypoint for faster alpha computation
	 */
	double lookahead_distance = params.factor_lookahead_distance() * velocity_;

	// angle between vehicle theta and the connection between the rear axis and the look ahead point
	const double alpha = computeAlpha(lookahead_distance, pose);

	const double delta = atan2(2. * params.vehicle_length() * sin(alpha), lookahead_distance);

	//	 const double delta = asin((VEHICLE_LENGTH * alpha) / lookahead_distance);

	move_cmd.setDirection( delta);
	move_cmd.setVelocity( velocity_);

	ROS_INFO("Command: vel=%f, angle=%f", velocity_, delta);

	*cmd = move_cmd;

	// visualize driven path
	geometry_msgs::Point p;
	p.x = pose[0];
	p.y = pose[1];
	path_marker.points.push_back(p);

	visualizer->getMarkerPublisher().publish(path_marker);

	return RobotController::MoveCommandStatus::OKAY;
}

void Robotcontroller_Ackermann_PurePursuit::publishMoveCommand(
		const MoveCommand& cmd) const {

	geometry_msgs::Twist msg;
	msg.linear.x  = cmd.getVelocity();
	msg.linear.y  = 0;
	msg.angular.z = cmd.getDirectionAngle();

	cmd_pub_.publish(msg);
}

void Robotcontroller_Ackermann_PurePursuit::publishInterpolatedPath() const {
	path_interpol_pub.publish((nav_msgs::Path) path_interpol);
}

void Robotcontroller_Ackermann_PurePursuit::initialize() {
	initialized = true;
}

bool Robotcontroller_Ackermann_PurePursuit::reachedGoal(
		const Eigen::Vector3d& pose) const {
	const unsigned int end = path_interpol.length() - 1;
	return hypot(path_interpol.p(end) - pose[0], path_interpol.q(end) - pose[1])
			<= params.goal_tolerance();
}

double Robotcontroller_Ackermann_PurePursuit::computeAlpha(
		double& lookahead_distance, const Eigen::Vector3d& pose) const {

	// TODO: correct angle, when the goal is near

	for (int j = path_interpol.length() - 1; j >= 0; --j) {
		unsigned int i = j;

		// TODO: test if this is necessary:
		//		const double angle_diff = MathHelper::AngleDelta(pose[2], path_interpol.theta_p(i));
		// only take points into account that have approximately the same angle (difference <= 90°)
		if (true) {//angle_diff <= M_PI_2 && angle_diff >= -M_PI_2) {


			const double dx = path_interpol.p(i) - pose[0];
			const double dy = path_interpol.q(i) - pose[1];

			const double distance = hypot(dx, dy);
			if (distance <= lookahead_distance) {

				// angle between the connection line and the vehicle orientation
				const double alpha = MathHelper::AngleDelta(pose[2], atan2(dy, dx));

				// set lookahead_distance to the actual distance
				lookahead_distance = distance;


				// line to lookahead point
				geometry_msgs::Point from, to;
				from.x = pose[0]; from.y = pose[1];
				to.x = path_interpol.p(i); to.y = path_interpol.q(i);

				visualizer->drawLine(12341234, from, to, "map", "geo", 1, 0, 0, 1, 0.01);

#ifdef DEBUG
				ROS_INFO("LookAheadPoint: index=%i, x=%f, y=%f", i, path_interpol.p(i), path_interpol.q(i));
				ROS_INFO("Pose: x=%f, y=%f, theta=%f", pose[0], pose[1], pose[2]);
				ROS_INFO("Alpha=%f", alpha);
#endif

				return alpha;
			}
		}
	}

	ROS_WARN("No appropriate path point found");

	return 0.;


}
