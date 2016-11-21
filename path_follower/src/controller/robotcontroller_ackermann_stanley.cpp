#include <path_follower/controller/robotcontroller_ackermann_stanley.h>



#include <path_follower/utils/pose_tracker.h>
#include <ros/ros.h>
#include <path_follower/utils/visualizer.h>

#include <interpolation.h>
#include <cslibs_utils/MathHelper.h>

#include <visualization_msgs/Marker.h>

#include <deque>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <limits>
#include <boost/algorithm/clamp.hpp>

RobotController_Ackermann_Stanley::RobotController_Ackermann_Stanley() :
    RobotController_Interpolation() {

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

    RobotController_Interpolation::reset();
}

void RobotController_Ackermann_Stanley::setPath(Path::Ptr path) {
    RobotController_Interpolation::setPath(path);

    // decide whether to drive forward or backward
    if (path_->getCurrentSubPath().forward) {
        setDirSign(1.f);
    } else {
        setDirSign(-1.f);
    }
}

RobotController::MoveCommandStatus RobotController_Ackermann_Stanley::computeMoveCommand(
		MoveCommand* cmd) {

	if(path_interpol.n() <= 2)
		return RobotController::MoveCommandStatus::ERROR;

    const Eigen::Vector3d pose = pose_tracker_->getRobotPose();

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
    visualizer_->drawLine(12341234, from, to, getFixedFrame(), "kinematic", 1, 0, 0, 1, 0.01);


	// distance to the path (path to the right -> positive)
	Eigen::Vector2d path_vehicle(pose[0] - path_interpol.p(ind), pose[1] - path_interpol.q(ind));

	double d =
			MathHelper::AngleDelta(MathHelper::Angle(path_vehicle), path_interpol.theta_p(ind)) < 0. ?
				-min_dist : min_dist;


	// theta_e = theta_vehicle - theta_path (orientation error)
	double theta_e = MathHelper::AngleDelta(pose[2], path_interpol.theta_p(ind));

	// if |theta_e| > 90° we drive backwards
	if (theta_e > M_PI_2) {
		setDirSign(-1.f);
		d = -d;
		theta_e = M_PI - theta_e;
	} else if (theta_e < -M_PI_2) {
		setDirSign(-1.f);
		d = -d;
		theta_e = -M_PI - theta_e;
	} else {
		setDirSign(1.f);
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
