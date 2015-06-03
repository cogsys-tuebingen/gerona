#include <path_follower/legacy/robotcontroller_ackermann_kinematic.h>
#include <path_follower/pathfollower.h>
#include <ros/ros.h>

#include "../alglib/interpolation.h"
#include <utils_general/MathHelper.h>

#include <visualization_msgs/Marker.h>

#include <deque>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <limits>

RobotController_Ackermann_Kinematic::RobotController_Ackermann_Kinematic(PathFollower* _path_follower) :
    RobotController_Interpolation(_path_follower) {

	path_interpol_pub = node_handle.advertise<nav_msgs::Path>("interp_path", 10);

	const double k = params.k();
	k1 = k * k * k;
	k2 = 3. * k * k;
	k3 = 3. * k;

	delta = 0.;

	ROS_INFO("Parameters: vehicle_length=%f, k=%f", params.vehicle_length(), params.k());

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
	oldTime = ros::Time::now();
}

RobotController::MoveCommandStatus RobotController_Ackermann_Kinematic::computeMoveCommand(
		MoveCommand* cmd) {

	ROS_INFO("===============================");
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


	double minDist = std::numeric_limits<double>::max();
	unsigned int s = 0;
	for (unsigned int i = 0; i < path_interpol.length(); ++i) {
		const double dx = path_interpol.p(i) - pose[0];
		const double dy = path_interpol.q(i) - pose[1];

		const double distance = hypot(dx, dy);
		if (distance < minDist) {
			minDist = distance;
			s = i;
		}
	}

	// line to nearest waypoint
	geometry_msgs::Point from, to;
	from.x = pose[0]; from.y = pose[1];
	to.x = path_interpol.p(s); to.y = path_interpol.q(s);

    visualizer_->drawLine(12341234, from, to, "map", "kinematic", 1, 0, 0, 1, 0.01);


	// distance to the path (path to the right -> positive)
	Eigen::Vector2d pathVehicle(pose[0] - path_interpol.p(s),
			 pose[1] - path_interpol.q(s));

	const double errorRearAxis = MathHelper::Angle(pathVehicle) > path_interpol.theta_p(s) ?
				minDist : -minDist;


	// TODO: must be != M_PI2 or -M_PI2
	const double errorTheta = MathHelper::AngleDelta(path_interpol.theta_p(s), pose[2]);

	const double curvature = path_interpol.curvature(s);
	const double curvature_prim = path_interpol.curvature_prim(s);
	const double curvature_sek = path_interpol.curvature_sek(s);

	ROS_INFO("e_ra=%f, e_theta=%f, c=%f, c'=%f", errorRearAxis, errorTheta, curvature, curvature_prim);

	// 1 - e_ra * k(s)
	const double curvatureError = 1. - errorRearAxis * curvature;

	// cos, sin, tan of theta error
	const double cosErrorTheta = cos(errorTheta);
	const double cosErrorTheta2 = cosErrorTheta * cosErrorTheta;
	const double cosErrorTheta3 = cosErrorTheta2 * cosErrorTheta;

	const double sinErrorTheta = sin(errorTheta);
	const double sinErrorTheta2 = sinErrorTheta * sinErrorTheta;

	const double tanErrorTheta = tan(errorTheta);
	const double tanErrorTheta2 = tanErrorTheta * tanErrorTheta;

	//	const double x1 = s;
	const double x2_1 = -curvature_prim * errorRearAxis * tanErrorTheta;
	const double x2_2 = curvature * curvatureError
			* ((1. + sinErrorTheta2) / cosErrorTheta2);
	const double x2_3 = pow(curvatureError, 2) * tan(delta)
			/ (params.vehicle_length() * cosErrorTheta3);

	const double x2 = x2_1 - x2_2 + x2_3;
	const double x3 = curvatureError * tanErrorTheta;
	const double x4 = errorRearAxis;

	// u1 is taken from "Feedback control for a path following robotic car" by Mellodge,
	// p. 108 (u1_actual)
	const double u1 = velocity_ * cosErrorTheta / curvatureError;
	const double u2 = -k1 * abs(u1) * x2 - k2 * u1 * x3 - k3 * abs(u1) * x4;
//	const double u2 = -k1 * u1 * x2 - k2 * u1 * x3 - k3 * u1 * x4;

	// alpha1: also from Mellodge
//	const double dx2derrorRearAxis =
//			curvature * curvature * (1 + sinErrorTheta2) / cosErrorTheta2
//			- 2. * curvatureError * curvature * tan(delta)
//			/ (params.vehicle_length() * cosErrorTheta3);

//	const double dx2dThetaError =
//			-curvature * curvatureError * 4. * tanErrorTheta / cosErrorTheta2
//			+ 3. * pow(curvatureError, 2) * tan(delta) * tanErrorTheta
//			/ (params.vehicle_length() * cosErrorTheta3);

	// this slightly differs from de Luca: dx2/ds is missing
//	const double dx2ds = 0.;

	// my version
	const double dx2derrorRearAxis =
			curvature_prim * tanErrorTheta
			- curvature * curvature * (1 + sinErrorTheta2) / cosErrorTheta2
			- 2. * curvatureError * curvature * tan(delta)
			/ (params.vehicle_length() * cosErrorTheta3);

	// (why 4????)
	const double dx2derrorTheta =
			curvature_prim * (tanErrorTheta2 + 1.)
			- 2. * curvature * curvatureError * tanErrorTheta / cosErrorTheta2
			+ 3. * pow(curvatureError, 2) * tan(delta) * tanErrorTheta / (params.vehicle_length()
																							  * cosErrorTheta3);

	const double dx2ds = curvature_sek * errorRearAxis * tanErrorTheta
			+ curvatureError * errorRearAxis * curvature
			* (pow(curvature_prim, 2) * (1 + sinErrorTheta2) / cosErrorTheta2
				- 2. * tan(delta) / (params.vehicle_length() * cosErrorTheta3));

	const double alpha1 =
			dx2ds
			+ dx2derrorRearAxis * curvatureError * tanErrorTheta
			+ dx2derrorTheta * (tan(delta) * curvatureError / (params.vehicle_length() * cosErrorTheta)
									- curvature);

	// alpha2:
	const double alpha2 =
			params.vehicle_length() * cosErrorTheta3 * pow(cos(delta), 2) / pow(curvatureError, 2);

	// longitudinal velocity
	double vel = (curvatureError / cosErrorTheta) * u1;
	if (vel > velocity_)
		vel = velocity_;

	// steering angle velocity
	const double velDelta = alpha2 * (u2 - alpha1 * u1);
	ROS_INFO("alpha1=%f, alpha2=%f, u1=%f, u2=%f", alpha1, alpha2, u1, u2);

	// update delta according to the time that has passed since the last update
	ros::Duration timePassed = ros::Time::now() - oldTime;
	delta += velDelta * timePassed.toSec();
	oldTime = ros::Time::now();


	ROS_INFO("Command: velocity=%f, angle=%f, velDelta=%f", vel, delta, velDelta);
	move_cmd.setDirection((float) delta);
	move_cmd.setVelocity((float) vel);


	*cmd = move_cmd;

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

bool RobotController_Ackermann_Kinematic::reachedGoal(
		const Eigen::Vector3d& pose) const {
	const unsigned int end = path_interpol.length() - 1;
	return hypot(path_interpol.p(end) - pose[0], path_interpol.q(end) - pose[1])
			<= params.goal_tolerance();
}



























