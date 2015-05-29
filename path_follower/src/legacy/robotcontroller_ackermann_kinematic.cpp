#include <path_follower/legacy/robotcontroller_ackermann_kinematic.h>
#include <path_follower/pathfollower.h>
#include <ros/ros.h>

#include "../alglib/interpolation.h"
#include <utils_general/MathHelper.h>

#include <visualization_msgs/Marker.h>

#include <deque>
#include <Eigen/Core>
#include <Eigen/Dense>


RobotController_Ackermann_Kinematic::RobotController_Ackermann_Kinematic(PathFollower* _path_follower) :
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

	const double k = params.k();
	k1 = k * k * k;
	k2 = 3. * k * k;
	k3 = 3. * k;

	ROS_INFO("Parameters: vehicle_length=%f", params.vehicle_length());

}

void RobotController_Ackermann_Kinematic::stopMotion() {

	move_cmd.setVelocity(0.f);
	move_cmd.setDirection(0.f);

	MoveCommand cmd = move_cmd;
	publishMoveCommand(cmd);
}

void RobotController_Ackermann_Kinematic::start() {
	path_driver_->getCoursePredictor().reset();
}

void RobotController_Ackermann_Kinematic::reset() {
	initialized = false;
	oldTime = ros::Time::now();
}

void RobotController_Ackermann_Kinematic::setPath(Path::Ptr path) {

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

RobotController::MoveCommandStatus RobotController_Ackermann_Kinematic::computeMoveCommand(
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


	double minDist = 0.;
	unsigned int minIndex = 0;
	for (unsigned int i = 0; i < path_interpol.length(); ++i) {
		const double dx = path_interpol.p(i) - pose[0];
		const double dy = path_interpol.q(i) - pose[1];

		const double distance = hypot(dx, dy);
		if (distance < minDist) {
			minDist = distance;
			minIndex = i;
		}
	}

	const double errorRearAxis = minDist;
	const double errorTheta = MathHelper::AngleDelta(pose[2], path_interpol.theta_p(minIndex));

	const double curvature = path_interpol.curvature(minIndex);
	// TODO curvature'
	const double curvature_prim = 0.;

	// 1 - e_ra * k(s)
	const double curvatureError = 1. - errorRearAxis * curvature;

	// cos, sin, tan of theta error
	const double cosErrorTheta = cos(errorTheta);
	const double cosErrorTheta2 = cosErrorTheta * cosErrorTheta;
	const double cosErrorTheta3 = cosErrorTheta2 * cosErrorTheta;

	const double sinErrorTheta = sin(errorTheta);
	const double sinErrorTheta2 = sinErrorTheta * sinErrorTheta;

	const double tanErrorTheta = tan(errorTheta);

	//	const double x1 = s;
	const double x2_1 = -curvature_prim * errorRearAxis * tanErrorTheta;
	const double x2_2 = curvature * curvatureError
			* ((1. + sinErrorTheta2) / cosErrorTheta2);
	const double x2_3 = (pow(curvatureError, 2) * tan(delta))
			/ (params.vehicle_length() * cosErrorTheta2);

	const double x2 = x2_1 - x2_2 + x2_3;
	const double x3 = curvatureError * tanErrorTheta;
	const double x4 = errorRearAxis;

	// u1 is taken from "Feedback control for a path following robotic car" by Mellodge,
	// p. 108 (u1_actual)
	const double u1 = velocity_ * cosErrorTheta / curvatureError;
	const double u2 = -k1 * abs(u1) * x2 - k2 * u1 * x3 - k3 * abs(u1) * x4;

	// alpha1: also from Mellodge
	const double dx2derrorRearAxis =
			curvature * curvature * (1 + sinErrorTheta2) / cosErrorTheta2
			- 2. * curvatureError * curvature * tan(delta)
			/ (params.vehicle_length() * cosErrorTheta2);

	const double dx2dThetaError =
			-curvature * curvatureError * 4. * tanErrorTheta / cosErrorTheta2
			+ 3. * pow(curvatureError, 2) * tan(delta) * tanErrorTheta
			/ (params.vehicle_length() * cosErrorTheta2);

	// this slightly differs from de Luca: dx2/ds is missing
	const double alpha1 =
			dx2derrorRearAxis * curvatureError * tanErrorTheta
			+ dx2dThetaError * (tan(delta) * curvatureError / (params.vehicle_length() * cosErrorTheta)
									- curvature);

	// alpha2:
	const double alpha2 =
			params.vehicle_length() * cosErrorTheta3 * pow(cos(delta), 2) / pow(curvatureError, 2);

	// longitudinal velocity
	const double velocity_ = (curvatureError / cosErrorTheta) * u1;
	// steering angle velocity
	const double velDelta = alpha2 * (u2 - alpha1 * u1);

	// update delta according to the time that has passed since the last update
	ros::Duration timePassed = ros::Time::now() - oldTime;
	delta += velDelta * timePassed.toSec();

	oldTime = ros::Time::now();

	move_cmd.setDirection((float) delta);
	move_cmd.setVelocity((float) velocity_);

	ROS_INFO("Command: vel=%f, angle=%f, velDelta=%f", velocity_, delta, velDelta);

	*cmd = move_cmd;

	// visualize driven path
	geometry_msgs::Point p;
	p.x = pose[0];
	p.y = pose[1];
	path_marker.points.push_back(p);

	visualizer->getMarkerPublisher().publish(path_marker);


	return RobotController::MoveCommandStatus::OKAY;
}

void RobotController_Ackermann_Kinematic::publishMoveCommand(
		const MoveCommand& cmd) const {

	geometry_msgs::Twist msg;
	msg.linear.x  = cmd.getVelocity();
	msg.linear.y  = 0;
	msg.angular.z = cmd.getDirectionAngle();

	cmd_pub_.publish(msg);
}

void RobotController_Ackermann_Kinematic::publishInterpolatedPath() const {
	path_interpol_pub.publish((nav_msgs::Path) path_interpol);
}

void RobotController_Ackermann_Kinematic::initialize() {
	initialized = true;
}

bool RobotController_Ackermann_Kinematic::reachedGoal(
		const Eigen::Vector3d& pose) const {
	const unsigned int end = path_interpol.length() - 1;
	return hypot(path_interpol.p(end) - pose[0], path_interpol.q(end) - pose[1])
			<= params.goal_tolerance();
}



























