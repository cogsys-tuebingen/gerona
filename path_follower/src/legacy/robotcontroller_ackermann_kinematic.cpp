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
#include <boost/algorithm/clamp.hpp>

RobotController_Ackermann_Kinematic::RobotController_Ackermann_Kinematic(PathFollower* _path_follower) :
	RobotController_Interpolation(_path_follower) {

	path_interpol_pub = node_handle.advertise<nav_msgs::Path>("interp_path", 10);

	const double k = params.k_forward();
	setTuningParameters(k);

	delta = 0.;

	ROS_INFO("Parameters: k_forward=%f, k_backward=%f\nvehicle_length=%f\nfactor_steering_angle=%f"
				"\ngoal_tolerance=%f\nmax_steering_angle=%f",
				params.k_forward(), params.k_backward(),
				params.vehicle_length(), params.factor_steering_angle(),
				params.goal_tolerance(), params.max_steering_angle());

}

void RobotController_Ackermann_Kinematic::setTuningParameters(const double k) {
	k1 = k * k * k;
	//	k1 = 10. * k * k;
	k2 = 3. * k * k;
	k3 = 3. * k;
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
	RobotController_Interpolation::reset();
}

RobotController::MoveCommandStatus RobotController_Ackermann_Kinematic::computeMoveCommand(
		MoveCommand* cmd) {

	ROS_INFO("===============================");

	if(path_interpol.n() <= 2)
		return RobotController::MoveCommandStatus::ERROR;

	const Eigen::Vector3d pose = path_driver_->getRobotPose();

	// TODO: theta should also be considered in goal test
	if (reachedGoal(pose)) {
		path_->switchToNextSubPath();
		if (path_->isDone()) {
			move_cmd.setDirection(0.);
			move_cmd.setVelocity(0.);

			*cmd = move_cmd;

#ifdef DEBUG
			ROS_INFO("Reached goal.");
#endif

			return RobotController::MoveCommandStatus::REACHED_GOAL;

		} else {

			ROS_INFO("Next subpath...");

			try {
				path_interpol.interpolatePath(path_);
				//				 publishInterpolatedPath();

			} catch(const alglib::ap_error& error) {
				throw std::runtime_error(error.msg);
			}
		}
	}


	double minDist = std::numeric_limits<double>::max();
	unsigned int j = 0;
	for (unsigned int i = 0; i < path_interpol.n(); ++i) {
		const double dx = path_interpol.p(i) - pose[0];
		const double dy = path_interpol.q(i) - pose[1];

		const double dist = hypot(dx, dy);
		if (dist < minDist) {
			minDist = dist;
			j = i;
		}
	}

	// line to nearest waypoint
	geometry_msgs::Point from, to;
	from.x = pose[0]; from.y = pose[1];
	to.x = path_interpol.p(j); to.y = path_interpol.q(j);

	visualizer_->drawLine(12341234, from, to, "map", "kinematic", 1, 0, 0, 1, 0.01);


	// distance to the path (path to the right -> positive)
	Eigen::Vector2d pathVehicle(pose[0] - path_interpol.p(j), pose[1] - path_interpol.q(j));

	double d = MathHelper::AngleDelta(MathHelper::Angle(pathVehicle), path_interpol.theta_p(j)) < 0. ?
				minDist : -minDist;


	// TODO: must be != M_PI_2 or -M_PI_2
	double thetaP = MathHelper::AngleDelta(path_interpol.theta_p(j), pose[2]);

	// TODO: decide whether to drive forward or backward
	if (thetaP > M_PI_2) {
		setDirSign(-1.f);
		d = -d;
		thetaP = M_PI - thetaP;
		setTuningParameters(params.k_backward());
	} else if (thetaP < -M_PI_2) {
		setDirSign(-1.f);
		d = -d;
		thetaP = -M_PI - thetaP;
		setTuningParameters(params.k_backward());
	} else {
		setTuningParameters(params.k_forward());
		setDirSign(1.f);
	}

	const double c = path_interpol.curvature(j);
	const double c_prim = path_interpol.curvature_prim(j);
	const double c_sek = path_interpol.curvature_sek(j);

	ROS_INFO("d=%f, thetaP=%f, c=%f, c'=%f, c''=%f", d, thetaP, c, c_prim, c_sek);

	const unsigned int j_1 = j == path_interpol.n() - 1 ? j : j + 1;
	const unsigned int j_0 = j_1 - 1;
	const double delta_s = path_interpol.s(j_1) - path_interpol.s(j_0);

	// d'
	double d_prim = hypot(pose[0] - path_interpol.p(j_1),	pose[1] - path_interpol.q(j_1));
	Eigen::Vector2d pathVehiclePlus1(pose[0] - path_interpol.p(j_1),
			pose[1] - path_interpol.q(j_1));

	d_prim =
			MathHelper::AngleDelta(MathHelper::Angle(pathVehiclePlus1), path_interpol.theta_p(j_1)) < 0. ?
				d_prim : -d_prim;

	d_prim = (d_prim - d) / delta_s;

	// thetaP'
	const double thetaP_prim =
			MathHelper::AngleDelta(thetaP,
										  MathHelper::AngleDelta(path_interpol.theta_p(j_1), pose[2]))
			/ delta_s;

	ROS_INFO("d'=%f, thetaP'=%f", d_prim, thetaP_prim);

	// 1 - dc(s)
	const double curvatureError = 1. - d * c; // OK

	// cos, sin, tan of theta error
	const double cosThetaP = cos(thetaP);  // OK
	const double cosThetaP2 = cosThetaP * cosThetaP; // OK
	const double cosThetaP3 = cosThetaP2 * cosThetaP; // OK

	const double sinThetaP = sin(thetaP); // OK
	const double sinThetaP2 = sinThetaP * sinThetaP; // OK

	const double tanThetaP = tan(thetaP); // OK
	const double tanThetaP2 = tanThetaP * tanThetaP; // OK

	//	const double x1 = s;
	const double x2 = -c_prim * d * tanThetaP
			- c * curvatureError * (1. + sinThetaP2) / cosThetaP2
			+ pow(curvatureError, 2) * tan(delta) / (params.vehicle_length() * cosThetaP3); // OK

	const double x3 = curvatureError * tanThetaP; // OK
	const double x4 = d; // OK

	// u1 is taken from "Feedback control for a path following robotic car" by Mellodge,
	// p. 108 (u1_actual)
	const double u1 = velocity_ * cosThetaP / curvatureError; // OK
	//	const double u2 = -k1 * abs(u1) * x2 - k2 * u1 * x3 - k3 * abs(u1) * x4;
	const double u2 = -k1 * u1 * x4 - k2 * u1 * x3 - k3 * u1 * x2; // OK

	//alpha1: also from Mellodge
	//	const double dx2dd =
	//			c * c * (1 + sinThetaP2) / cosThetaP2
	//			- 2. * curvatureError * c * tan(delta) / (params.vehicle_length() * cosThetaP3); // OK

	//	const	double dx2dthetaP =
	//			-c * curvatureError * 4. * tanThetaP / cosThetaP2
	//			+ 3. * pow(curvatureError, 2) * tan(delta) * tanThetaP / (params.vehicle_length() * cosThetaP3); // OK

	//	this slightly differs from de Luca: dx2/ds is missing
	//	const double dx2ds = 0.; // OK

	// my version
	const double dx2dd = c_prim * tanThetaP
			- c * c * (1 + sinThetaP2) / cosThetaP2
			- 2. * curvatureError * c * tan(delta) / (params.vehicle_length() * cosThetaP3);

	const double dx2dthetaP = c_prim * (tanThetaP2 + 1.)
			- 4. * c * curvatureError * tanThetaP / cosThetaP2
			+ 3. * pow(curvatureError, 2) * tan(delta) * tanThetaP / (params.vehicle_length()
																						 * cosThetaP3);

//	const double dx2ds = c_sek * d * tanThetaP
//			+ curvatureError * d * c * (pow(c_prim, 2) * (1. + sinThetaP2) / cosThetaP2
//												 - 2. * tan(delta) / (params.vehicle_length() * cosThetaP3));

	//	const double dx2ds =
	//			tanThetaP * (c_sek * d + c_prim * d_prim)
	//			+ c_prim * d * thetaP_prim * (tanThetaP2 + 1.)
	//			- curvatureError * (c_prim * (1. + sinThetaP2) / cosThetaP2
	//									  + 2. * c * thetaP_prim * tanThetaP / cosThetaP2)
	//			- c * (d_prim * c + d * c_prim) * (1. + sinThetaP2) / cosThetaP2
	//			+ (tan(delta) * curvatureError / params.vehicle_length())
	//			* (2. * (d_prim * c + d * c_prim) * cosThetaP2
	//				+ 3. * thetaP_prim * curvatureError * sinThetaP)
	//			/ (cosThetaP2 * cosThetaP3);

	const double dx2ds =
			tanThetaP * (c_sek * d + c_prim * d_prim)
			+ c_prim * d * thetaP_prim * (1. + tanThetaP2)
			- ((1. + sinThetaP2) / cosThetaP2) * (c_prim * curvatureError + c * (d_prim * c + d * c_prim))
			- 4. * c * curvatureError * tanThetaP / cosThetaP2
			+ (curvatureError * tan(delta) / params.vehicle_length())
			* (-2. * (d_prim * c + d * c_prim)
				+ thetaP_prim * curvatureError + sinThetaP)
			/ pow(cosThetaP2, 2);

	const double alpha1 =
			dx2ds
			+ dx2dd * curvatureError * tanThetaP
			+ dx2dthetaP * (tan(delta) * curvatureError / (params.vehicle_length() * cosThetaP) - c); // OK

	// alpha2:
	const double alpha2 =
			params.vehicle_length() * cosThetaP3 * pow(cos(delta), 2) / pow(curvatureError, 2); // OK

	ROS_INFO("alpha1=%f, alpha2=%f, u1=%f, u2=%f", alpha1, alpha2, u1, u2);

	// longitudinal velocity
	double v1 = curvatureError * u1 / cosThetaP; // OK
	if (v1 > velocity_)
		v1 = velocity_;

	// steering angle velocity
	const double v2 = alpha2 * (u2 - alpha1 * u1); // OK

	// update delta according to the time that has passed since the last update
	ros::Duration timePassed = ros::Time::now() - oldTime;
	delta += v2 * timePassed.toSec();
	oldTime = ros::Time::now();

	delta = boost::algorithm::clamp(delta, -params.max_steering_angle(), params.max_steering_angle());

	ROS_INFO("Time passed: %fs, command: v1=%f, v2=%f, delta=%f",
				timePassed.toSec(), v1, v2, delta);
	move_cmd.setDirection(params.factor_steering_angle() * (float) delta);
	move_cmd.setVelocity(getDirSign() * (float) v1);


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



























