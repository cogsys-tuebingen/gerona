#include <path_follower/controller/robotcontroller_2steer_inputscaling.h>


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <path_follower/utils/pose_tracker.h>
#include <path_follower/utils/visualizer.h>

#include <cslibs_utils/MathHelper.h>

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

#include <path_follower/factory/controller_factory.h>

REGISTER_ROBOT_CONTROLLER(RobotController_2Steer_InputScaling, 2steer_inputscaling, default_collision_avoider);

RobotController_2Steer_InputScaling::RobotController_2Steer_InputScaling() :
    RobotController(),
	phi_(0.)
{

	const double k = params_.k_forward();
	setTuningParameters(k);

	ROS_INFO("Parameters: k_forward=%f, k_backward=%f\n"
				"vehicle_length=%f\n"
				"goal_tolerance=%f\n"
				"max_steering_angle=%f\nmax_steering_angle_speed=%f",
				params_.k_forward(), params_.k_backward(),
				params_.vehicle_length(),
				params_.goal_tolerance(),
				params_.max_steering_angle(), params_.max_steering_angle_speed());

#ifdef TEST_OUTPUT
	test_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/test_output", 100);
#endif
}

void RobotController_2Steer_InputScaling::setTuningParameters(const double k) {
	k1_ = 1. * k * k * k;
	k2_ = 3. * k * k;
	k3_ = 3. * k;
}

void RobotController_2Steer_InputScaling::stopMotion() {

	move_cmd_.setVelocity(0.f);
	move_cmd_.setDirection(0.f);

	phi_ = 0.;

	MoveCommand cmd = move_cmd_;
	publishMoveCommand(cmd);
}

void RobotController_2Steer_InputScaling::start() {

}

void RobotController_2Steer_InputScaling::reset() {
	old_time_ = ros::Time::now();

    RobotController::reset();
}

void RobotController_2Steer_InputScaling::setPath(Path::Ptr path) {
    RobotController::setPath(path);

    if (getDirSign() < 0.)
        setTuningParameters(params_.k_backward());
    else
        setTuningParameters(params_.k_forward());
}

RobotController::MoveCommandStatus RobotController_2Steer_InputScaling::computeMoveCommand(
		MoveCommand* cmd) {

	std::clock_t begin = std::clock();

	if(path_interpol.n() <= 2)
		return RobotController::MoveCommandStatus::ERROR;

    const Eigen::Vector3d pose = pose_tracker_->getRobotPose();
    const geometry_msgs::Twist v_meas_twist = pose_tracker_->getVelocity();

    double velocity_measured = dir_sign_ * sqrt(v_meas_twist.linear.x * v_meas_twist.linear.x
            + v_meas_twist.linear.y * v_meas_twist.linear.y);

    RobotController::findOrthogonalProjection();
    double d = orth_proj_;

    if(RobotController::isGoalReached(cmd)){
       return RobotController::MoveCommandStatus::REACHED_GOAL;
    }

    if (getDirSign() < 0.)
        setTuningParameters(params_.k_backward());
    else
        setTuningParameters(params_.k_forward());

	// draw a line to the orthogonal projection
	geometry_msgs::Point from, to;
	from.x = pose[0]; from.y = pose[1];
    to.x = path_interpol.p(proj_ind_); to.y = path_interpol.q(proj_ind_);
    visualizer_->drawLine(12341234, from, to, getFixedFrame(), "kinematic", 1, 0, 0, 1, 0.01);


	// theta_e = theta_vehicle - theta_path (orientation error)
    double theta_e = MathHelper::AngleDelta(path_interpol.theta_p(proj_ind_), pose[2]);

	// if dir_sign is negative we drive backwards and set theta_e to the complementary angle
	if (getDirSign() < 0.)
		theta_e = theta_e > 0.? -M_PI + theta_e : M_PI + theta_e;

	// curvature and first two derivations
    const double c = path_interpol.curvature(proj_ind_);
    const double dc_ds = path_interpol.curvature_prim(proj_ind_);
    const double dc_ds_2 = path_interpol.curvature_sek(proj_ind_);

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

	const double sin_phi = sin(phi_);

	//
	// actual controller formulas begin here
	//

	// x1 - x4
	//	const double x1 = s;
	const double x2 = -dc_ds * d * tan_theta_e
			- c * _1_dc * (1. + sin_theta_e_2) / cos_theta_e_2
			+ 2. * _1_dc_2 * sin_phi / (params_.vehicle_length() * cos_theta_e_3);

	const double x3 = _1_dc * tan_theta_e;
    const double x4 = d;


	// u1, u2

	const double u1 = velocity_ * cos_theta_e / _1_dc;
	const double u2 =
			- k1_ * fabs(u1) * x4
			- k2_ * u1 * x3
			- k3_ * fabs(u1) * x2;

	// derivations of x2 (for alpha1)
	const double dx2_dd = -dc_ds * tan_theta_e
			+ c * c * (1. + sin_theta_e_2) / cos_theta_e_2
			- 4. * _1_dc * c * sin_phi / (params_.vehicle_length() * cos_theta_e_3);

    const double dx2_dtheta_e = -dc_ds * d * (1. + tan_theta_e_2)
			- 4. * c * _1_dc * tan_theta_e / cos_theta_e_2
			+ 6. * _1_dc_2 * sin_phi * tan_theta_e / (params_.vehicle_length()
																	* cos_theta_e_3);
	const double dx2_ds =
			- dc_ds_2 * d * tan_theta_e
            - (dc_ds * _1_dc - d * dc_ds * c) * (1. + sin_theta_e_2) / cos_theta_e_2
            - 4. * _1_dc * dc_ds * d * sin_phi / (params_.vehicle_length() * cos_theta_e_3);

	// alpha1
	const double alpha1 =
			dx2_ds
			+ dx2_dd * _1_dc * tan_theta_e
			+ dx2_dtheta_e * (2. * sin_phi * _1_dc / (params_.vehicle_length() * cos_theta_e) - c);

	// alpha2
	const double alpha2 = params_.vehicle_length() * cos_theta_e_3 / (2. * _1_dc_2 * cos(phi_));

	// longitudinal velocity
	const double v1 = velocity_;

	// steering angle velocity
	double v2 = alpha2 * (u2 - alpha1 * u1);

	// limit steering angle velocity
	v2 = boost::algorithm::clamp(v2, -params_.max_steering_angle_speed(),
											params_.max_steering_angle_speed());

	// time
	const double time_passed = (ros::Time::now() - old_time_).toSec();
	old_time_ = ros::Time::now();

	// update delta according to the time that has passed since the last update
	phi_ += v2 * time_passed;

	// also limit the steering angle
	phi_ = boost::algorithm::clamp(phi_, -params_.max_steering_angle(), params_.max_steering_angle());

	move_cmd_.setDirection(getDirSign() * (float) phi_);
	move_cmd_.setVelocity(getDirSign() * (float) v1);
	*cmd = move_cmd_;


#ifdef TEST_OUTPUT
    publishTestOutput(proj_ind_, d, theta_e, phi_, velocity_measured);
#endif

	ROS_DEBUG("d=%f, theta_e=%f, c=%f, c'=%f, c''=%f", d, theta_e, c, dc_ds, dc_ds_2);
	ROS_DEBUG("1 - dc(s)=%f", _1_dc);
	ROS_DEBUG("dx2dd=%f, dx2dtheta_e=%f, dx2ds=%f", dx2_dd, dx2_dtheta_e, dx2_ds);
	ROS_DEBUG("alpha1=%f, alpha2=%f, u1=%f, u2=%f", alpha1, alpha2, u1, u2);
	ROS_DEBUG("frame time = %f", (((float) (std::clock() - begin)) / CLOCKS_PER_SEC));

	return RobotController::MoveCommandStatus::OKAY;
}

void RobotController_2Steer_InputScaling::publishMoveCommand(
		const MoveCommand& cmd) const {

	geometry_msgs::Twist msg;
	msg.linear.x  = cmd.getVelocity();
	msg.linear.y  = 0;
	msg.angular.z = cmd.getDirectionAngle();

	cmd_pub_.publish(msg);
}

#ifdef TEST_OUTPUT
void RobotController_2Steer_InputScaling::publishTestOutput(const unsigned int waypoint, const double d,
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
