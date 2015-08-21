#include <path_follower/legacy/robotcontroller_4ws_inputscaling.h>
#include <path_follower/pathfollower.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "../alglib/interpolation.h"
#include <utils_general/MathHelper.h>

#include <visualization_msgs/Marker.h>

#include <deque>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <limits>
#include <boost/algorithm/clamp.hpp>

#include <time.h>

#ifdef TEST_OUTPUT
#include <std_msgs/Float64MultiArray.h>
#endif

RobotController_4WS_InputScaling::RobotController_4WS_InputScaling(PathFollower* _path_follower) :
	RobotController_Interpolation(_path_follower),
	phi_(0.),
	v1_(0.),
	v2_(0.)
{

	const double k = params_.k_forward();
	setTuningParameters(k);

	ROS_INFO("Parameters: k_forward=%f, k_backward=%f\n"
				"factor_k1=%f, k2=%f, k3=%f\n"
				"vehicle_length=%f\n"
				"goal_tolerance=%f\nmax_steering_angle=%f",
				params_.k_forward(), params_.k_backward(),
				params_.factor_k1(), params_.factor_k2(), params_.factor_k3(),
				params_.vehicle_length(),
				params_.goal_tolerance(), params_.max_steering_angle());

#ifdef TEST_OUTPUT
	test_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/test_output", 100);
#endif
}

void RobotController_4WS_InputScaling::setTuningParameters(const double k) {
	k1_ = params_.factor_k1() * k * k * k;
	k2_ = params_.factor_k2() * k * k;
	k3_ = params_.factor_k3() * k;
}

void RobotController_4WS_InputScaling::stopMotion() {

	move_cmd_.setVelocity(0.f);
	move_cmd_.setDirection(0.f);

	phi_ = 0.;

	MoveCommand cmd = move_cmd_;
	publishMoveCommand(cmd);
}

void RobotController_4WS_InputScaling::start() {
	path_driver_->getCoursePredictor().reset();
}

void RobotController_4WS_InputScaling::reset() {
	old_time_ = ros::Time::now();

	v1_ = v2_ = 0.;
	s_prim_ = 0.001; // TODO: good starting value

	RobotController_Interpolation::reset();
}

void RobotController_4WS_InputScaling::setPath(Path::Ptr path) {
	RobotController_Interpolation::setPath(path);

	Eigen::Vector3d pose = path_driver_->getRobotPose();
	const double theta_diff = MathHelper::AngleDelta(path_interpol.theta_p(0), pose[2]);

	// decide whether to drive forward or backward
	if (theta_diff > M_PI_2 || theta_diff < -M_PI_2) {
		setDirSign(-1.f);
		setTuningParameters(params_.k_backward());
	} else {
		setDirSign(1.f);
		setTuningParameters(params_.k_forward());
	}
}

RobotController::MoveCommandStatus RobotController_4WS_InputScaling::computeMoveCommand(
		MoveCommand* cmd) {

	ROS_DEBUG("===============================");

	std::clock_t begin = std::clock();

	if(path_interpol.n() <= 2)
		return RobotController::MoveCommandStatus::ERROR;

	const Eigen::Vector3d pose = path_driver_->getRobotPose();
	const geometry_msgs::Twist velocity_measured = path_driver_->getVelocity();

	ROS_DEBUG("velocity_measured=%f", velocity_measured.linear.x);

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
			if (getDirSign() < 0.)
				setTuningParameters(params_.k_backward());
			else
				setTuningParameters(params_.k_forward());
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


	// theta_e = theta_vehicle - theta_path (orientation error)
	double theta_e = MathHelper::AngleDelta(path_interpol.theta_p(ind), pose[2]);

	// if dir_sign is negative we drive backwards and set theta_e to the complementary angle
	if (getDirSign() < 0.) {
		d = -d;
		setTuningParameters(params_.k_backward());
		theta_e = theta_e > 0.? M_PI - theta_e : -M_PI - theta_e;
	} else {
		setTuningParameters(params_.k_forward());
	}

	// curvature and first two derivations
	const double c = path_interpol.curvature(ind);
	const double dc_ds = path_interpol.curvature_prim(ind);
	const double c_sek = path_interpol.curvature_sek(ind);

	// 1 - dc(s)
	const double _1_dc = 1. - d * c;
	const double _1_dc_2 = _1_dc * _1_dc;


	// cos, sin, tan of theta_e and phi
	const double cos_theta_e = cos(theta_e);
	const double cos_theta_e_2 = cos_theta_e * cos_theta_e;
	const double cos_theta_e_3 = cos_theta_e_2 * cos_theta_e;

	const double sin_theta_e = sin(theta_e);
	const double sin_theta_e_2 = sin_theta_e * sin_theta_e;

	const double tan_theta_e = tan(theta_e);
	const double tan_theta_e_2 = tan_theta_e * tan_theta_e;

	const double tan_phi = tan(phi_);
	const double tan_phi_2 = tan_phi * tan_phi;

	// time
	const double time_passed = (ros::Time::now() - old_time_).toSec();
	old_time_ = ros::Time::now();

	// absolute measured velocity
	v1_ = max(abs(velocity_measured.linear.x), (double) velocity_);

	s_prim_ = cos_theta_e / _1_dc;

	// d', theta_e' and phi'
	const double dd_ds = sin_theta_e * v1_ / s_prim_;
	const double dtheta_e_ds = ((tan_phi / params_.vehicle_length() - c * cos_theta_e / _1_dc) * v1_)
			/ s_prim_;
	const double dphi_ds = v2_ / s_prim_;

	ROS_DEBUG("s_prim=%f, delta_s=%f", s_prim_, s_prim_ * time_passed);
	ROS_DEBUG("d'=%f, theta_e'=%f, phi'=%f", dd_ds, dtheta_e_ds, dphi_ds);

	//
	// actual controller formulas begin here
	//

	// x1 - x4
	//	const double x1 = s;
	const double x2 = -dc_ds * d * tan_theta_e
			- c * _1_dc * (1. + sin_theta_e_2) / cos_theta_e_2
			+ _1_dc_2 * tan_phi / (params_.vehicle_length() * cos_theta_e_3);

	const double x3 = _1_dc * tan_theta_e;
	const double x4 = d;

	// u1, u2
	// u1 is taken from "Feedback control for a path following robotic car" by Mellodge,
	// p. 108 (u1_actual)
	const double u1 = v1_ * cos_theta_e / _1_dc;
	const double u2 =
			- k1_ * u1 * x4
			- k2_ * u1 * x3
			- k3_ * u1 * x2;

	// derivations of x2 (for alpha1)
	const double dx2_dd = -dc_ds * tan_theta_e
			+ c * c * (1. + sin_theta_e_2) / cos_theta_e_2
			- 2. * _1_dc * c * tan_phi / (params_.vehicle_length() * cos_theta_e_3); // OK

	const double dx2_dtheta_e = -dc_ds * d * (1. + tan_theta_e_2)
			- 4. * c * _1_dc * tan_theta_e / cos_theta_e_2
			+ 3. * _1_dc_2 * tan_phi * tan_theta_e / (params_.vehicle_length()
																				 * cos_theta_e_3); // OK
//	const double dx2_ds =
//			-tan_theta_e * (c_sek * d + dc_ds * dd_ds)
//			- dc_ds * d * dtheta_e_ds * (1. + tan_theta_e_2)
//			- (dc_ds * _1_dc - c * (dd_ds * c + d * dc_ds)) * ((1. + sin_theta_e_2) / cos_theta_e_2)
//			- 4. * c * _1_dc * dtheta_e_ds * tan_theta_e / cos_theta_e_2
//			+ (_1_dc * tan_phi / params_.vehicle_length())
//			* (-2. * (dd_ds * c + d * dc_ds) * cos_theta_e
//				+ 3. * dtheta_e_ds * _1_dc * sin_theta_e) / pow(cos_theta_e_2, 2); // OK

	const double dx2_ds =
			-tan_theta_e * (c_sek * d + dc_ds * dd_ds)
			- dc_ds * d * dtheta_e_ds * (1. + tan_theta_e_2)
			- (dc_ds * _1_dc - c * (dd_ds * c + d * dc_ds)) * ((1. + sin_theta_e_2) / cos_theta_e_2)
			- 4. * c * _1_dc * dtheta_e_ds * tan_theta_e / cos_theta_e_2
			+ (cos_theta_e * _1_dc * (-2. * (dd_ds * c + d * dc_ds) * tan_phi
											  +_1_dc * (1. + tan_phi_2) * dphi_ds)
				- 3. * dtheta_e_ds * sin_theta_e * _1_dc_2 * tan_phi)
			/ (params_.vehicle_length() * pow(cos_theta_e_2, 2)); // OK

	// alpha1
	const double alpha1 =
			dx2_ds
			+ dx2_dd * _1_dc * tan_theta_e
			+ dx2_dtheta_e * (tan_phi * _1_dc / (params_.vehicle_length() * cos_theta_e) - c);

	// alpha2
	const double alpha2 =
			params_.vehicle_length() * cos_theta_e_3 * pow(cos(phi_), 2) / _1_dc_2;


	// longitudinal velocity
	v1_ = velocity_; // u1 * _1_dc / cos_theta_e; // TODO: does this work?

	// steering angle velocity
	v2_ = alpha2 * (u2 - alpha1 * u1);

	// limit steering angle velocity
	v2_ = boost::algorithm::clamp(v2_, -params_.max_steering_angle_speed(), params_.max_steering_angle_speed());

	// update delta according to the time that has passed since the last update
	phi_ += v2_ * time_passed;

	// also limit the steering angle
	phi_ = boost::algorithm::clamp(phi_, -params_.max_steering_angle(), params_.max_steering_angle());

	ROS_DEBUG("d=%f, thetaP=%f, c=%f, c'=%f, c''=%f", d, theta_e, c, dc_ds, c_sek);
	ROS_DEBUG("d'=%f, thetaP'=%f", dd_ds, dtheta_e_ds);
	ROS_DEBUG("1 - dc(s)=%f", _1_dc);
	ROS_DEBUG("dx2dd=%f, dx2dthetaP=%f, dx2ds=%f", dx2_dd, dx2_dtheta_e, dx2_ds);
	ROS_DEBUG("alpha1=%f, alpha2=%f, u1=%f, u2=%f", alpha1, alpha2, u1, u2);
	ROS_DEBUG("Time passed: %fs, command: v1=%f, v2=%f, phi_=%f",
				 time_passed, v1_, v2_, phi_);

	// This is the accurate steering angle for 4 wheel steering
	const float delta = (float) asin(.5 * tan_phi);

	move_cmd_.setDirection(delta);
	move_cmd_.setVelocity(getDirSign() * (float) v1_);
	*cmd = move_cmd_;

#ifdef TEST_OUTPUT
	publishTestOutput(ind, d, theta_e, phi_, velocity_measured.linear.x);
#endif

	ROS_DEBUG("frame time = %f", (((float) (std::clock() - begin)) / CLOCKS_PER_SEC));

	return RobotController::MoveCommandStatus::OKAY;
}

void RobotController_4WS_InputScaling::publishMoveCommand(
		const MoveCommand& cmd) const {

	geometry_msgs::Twist msg;
	msg.linear.x  = cmd.getVelocity();
	msg.linear.y  = 0;
	msg.angular.z = cmd.getDirectionAngle();

	cmd_pub_.publish(msg);
}

#ifdef TEST_OUTPUT
void RobotController_4WS_InputScaling::publishTestOutput(const unsigned int waypoint, const double d,
																	 const double theta_e,
																	 const double phi, const double v) const {
	std_msgs::Float64MultiArray msg;

	msg.data.push_back((double) waypoint);
	msg.data.push_back(d);
	msg.data.push_back(theta_e);
	msg.data.push_back(phi);
	msg.data.push_back(v);

	test_pub_.publish(msg);
}
#endif
