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

	const double k = params_.k_forward();
	setTuningParameters(k);

	delta_ = 0.;

	ROS_INFO("Parameters: k_forward=%f, k_backward=%f\n"
				"factor_k1=%f, k2=%f, k3=%f\n"
				"vehicle_length=%f\n"
				"factor_steering_angle=%f\n"
				"goal_tolerance=%f\nmax_steering_angle=%f",
				params_.k_forward(), params_.k_backward(),
				params_.factor_k1(), params_.factor_k2(), params_.factor_k3(),
				params_.vehicle_length(),
				params_.factor_steering_angle(),
				params_.goal_tolerance(), params_.max_steering_angle());

}

void RobotController_Ackermann_Kinematic::setTuningParameters(const double k) {
	k1_ = params_.factor_k1() * k * k * k;
	k2_ = params_.factor_k2() * k * k;
	k3_ = params_.factor_k3() * k;
}

void RobotController_Ackermann_Kinematic::stopMotion() {

	move_cmd_.setVelocity(0.f);
	move_cmd_.setDirection(0.f);

	MoveCommand cmd = move_cmd_;
	publishMoveCommand(cmd);
}

void RobotController_Ackermann_Kinematic::start() {
	path_driver_->getCoursePredictor().reset();
}

void RobotController_Ackermann_Kinematic::reset() {
	old_time_ = ros::Time::now();
	RobotController_Interpolation::reset();
}

RobotController::MoveCommandStatus RobotController_Ackermann_Kinematic::computeMoveCommand(
		MoveCommand* cmd) {

	ROS_DEBUG("===============================");

	if(path_interpol.n() <= 2)
		return RobotController::MoveCommandStatus::ERROR;

	const Eigen::Vector3d pose = path_driver_->getRobotPose();

	// TODO: theta should also be considered in goal test
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
				min_dist : -min_dist;


	// theta_p = theta_vehicle - theta_path (orientation error)
	// TODO: must be != M_PI_2 or -M_PI_2
	double theta_p = MathHelper::AngleDelta(path_interpol.theta_p(ind), pose[2]);

	// if |theta_p| > 90° we drive backwards and set theta_p to the complementary angle
	if (theta_p > M_PI_2) {
		setDirSign(-1.f);
		d = -d;
		theta_p = M_PI - theta_p;
		setTuningParameters(params_.k_backward());
	} else if (theta_p < -M_PI_2) {
		setDirSign(-1.f);
		d = -d;
		theta_p = -M_PI - theta_p;
		setTuningParameters(params_.k_backward());
	} else {
		setTuningParameters(params_.k_forward());
		setDirSign(1.f);
	}


	// curvature and first two derivations
	const double c = path_interpol.curvature(ind);
	const double c_prim = path_interpol.curvature_prim(ind);
	const double c_sek = path_interpol.curvature_sek(ind);

	// indices to compute differential quotients for theta_p' and d'
	const unsigned int j_1 = ind == path_interpol.n() - 1 ? ind : ind + 1;
	const unsigned int j_0 = j_1 - 1;
	const double delta_s = path_interpol.s(j_1) - path_interpol.s(j_0);

	// theta_P'
	double theta_p_1 = MathHelper::AngleDelta(path_interpol.theta_p(j_1), pose[2]);

	if (theta_p_1 > M_PI_2)
		theta_p_1 = M_PI - theta_p_1;
	else if (theta_p_1 < -M_PI_2)
		theta_p_1 = -M_PI - theta_p_1;

	double theta_p_prim = MathHelper::AngleDelta(theta_p, theta_p_1) / delta_s;

	// d'
	Eigen::Vector2d path_vehicle_1(pose[0] - path_interpol.p(j_1), pose[1] - path_interpol.q(j_1));
	double d_prim = path_vehicle_1.norm();

	d_prim =
			MathHelper::AngleDelta(MathHelper::Angle(path_vehicle_1), path_interpol.theta_p(j_1)) < 0. ?
				d_prim : -d_prim;

	if (theta_p_1 > M_PI_2 || theta_p_1 < -M_PI_2)
		d_prim = -d_prim;

	d_prim = (d_prim - d) / delta_s;


	// 1 - dc(s)
	const double _1_dc = 1. - d * c;


	// cos, sin, tan of theta_p
	const double cos_theta_p = cos(theta_p);
	const double cos_theta_p_2 = cos_theta_p * cos_theta_p;
	const double cos_theta_p_3 = cos_theta_p_2 * cos_theta_p;

	const double sin_theta_p = sin(theta_p);
	const double sin_theta_p_2 = sin_theta_p * sin_theta_p;

	const double tan_theta_p = tan(theta_p);
	const double tan_theta_p_2 = tan_theta_p * tan_theta_p;



	//
	// actual controller formulas begin here
	//

	// x1 - x4
	//	const double x1 = s;
	const double x2 = -c_prim * d * tan_theta_p
			- c * _1_dc * (1. + sin_theta_p_2) / cos_theta_p_2
			+ pow(_1_dc, 2) * tan(delta_) / (params_.vehicle_length() * cos_theta_p_3);

	const double x3 = _1_dc * tan_theta_p;
	const double x4 = d;

	// u1, u2
	// u1 is taken from "Feedback control for a path following robotic car" by Mellodge,
	// p. 108 (u1_actual)
	const double u1 = velocity_ * cos_theta_p / _1_dc;
	const double u2 =
			- k1_ * u1 * x4
			- k2_ * u1 * x3
			- k3_ * u1 * x2;

	// derivations of x2 (for alpha1)
	const double dx2_dd = -c_prim * tan_theta_p
			+ c * c * (1 + sin_theta_p_2) / cos_theta_p_2
			- 2. * _1_dc * c * tan(delta_) / (params_.vehicle_length() * cos_theta_p_3);

	const double dx2_dtheta_p = -c_prim * (tan_theta_p_2 + 1.)
			- 4. * c * _1_dc * tan_theta_p / cos_theta_p_2
			+ 3. * pow(_1_dc, 2) * tan(delta_) * tan_theta_p / (params_.vehicle_length()
																				 * cos_theta_p_3);
	const double dx2_ds =
			-tan_theta_p * (c_sek * d + c_prim * d_prim)
			- c_prim * d * theta_p_prim * (1. + tan_theta_p_2)
			+ ((1. + sin_theta_p_2) / cos_theta_p_2) * (c_prim * _1_dc + c * (d_prim * c + d * c_prim))
			- 4. * c * _1_dc * tan_theta_p / cos_theta_p_2
			+ (_1_dc * tan(delta_) / params_.vehicle_length())
			* (-2. * (d_prim * c + d * c_prim)
				+ theta_p_prim * _1_dc + sin_theta_p) / pow(cos_theta_p_2, 2);

	// simple version where theta_p and d are considered independent of s
	//	const double dx2_ds = c_sek * d * tan_theta_p
	//			+ _1_dc * d * c * (pow(c_prim, 2) * (1. + sin_theta_p_2) / cos_theta_p_2
	//												 - 2. * tan(delta_) / (params_.vehicle_length() * cos_theta_p_3));

	// alpha1
	const double alpha1 =
			dx2_ds
			+ dx2_dd * _1_dc * tan_theta_p
			+ dx2_dtheta_p * (tan(delta_) * _1_dc / (params_.vehicle_length() * cos_theta_p) - c);

	// alpha2
	const double alpha2 =
			params_.vehicle_length() * cos_theta_p_3 * pow(cos(delta_), 2) / pow(_1_dc, 2);


	// longitudinal velocity
	double v1 = _1_dc * u1 / cos_theta_p;
	if (v1 > velocity_)
		v1 = velocity_;

	// steering angle velocity
	double v2 = alpha2 * (u2 - alpha1 * u1);

	// limit steering angle velocity
	v2 = boost::algorithm::clamp(v2, -params_.max_steering_angle_speed(), params_.max_steering_angle_speed());

	// update delta according to the time that has passed since the last update
	ros::Duration time_passed = ros::Time::now() - old_time_;
	delta_ += v2 * time_passed.toSec();
	old_time_ = ros::Time::now();

	ROS_DEBUG("d=%f, thetaP=%f, c=%f, c'=%f, c''=%f", d, theta_p, c, c_prim, c_sek);
	ROS_DEBUG("d'=%f, thetaP'=%f", d_prim, theta_p_prim);
	ROS_DEBUG("1 - dc(s)=%f", _1_dc);
	ROS_DEBUG("dx2dd=%f, dx2dthetaP=%f, dx2ds=%f", dx2_dd, dx2_dtheta_p, dx2_ds);
	ROS_DEBUG("alpha1=%f, alpha2=%f, u1=%f, u2=%f", alpha1, alpha2, u1, u2);
	ROS_DEBUG("Time passed: %fs, command: v1=%f, v2=%f, delta_=%f",
				 time_passed.toSec(), v1, v2, delta_);

	// This is the accurate steering angle for 4 wheel steering
	const float delta = (float) asin(params_.factor_steering_angle() * sin(delta_));

	ROS_INFO("delta=%f, delta_=%f", delta, delta_);

	move_cmd_.setDirection(delta);

	// also limit the steering angle
	delta_ = boost::algorithm::clamp(delta_, -params_.max_steering_angle(), params_.max_steering_angle());

	move_cmd_.setVelocity(getDirSign() * (float) v1);

	*cmd = move_cmd_;

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



























