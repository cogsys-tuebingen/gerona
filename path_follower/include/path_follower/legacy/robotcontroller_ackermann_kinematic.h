#ifndef ROBOTCONTROLLER_ACKERMANN_KINEMATIC_H
#define ROBOTCONTROLLER_ACKERMANN_KINEMATIC_H

#include <path_follower/controller/robotcontroller_interpolation.h>
#include <path_follower/utils/parameters.h>

#include <visualization_msgs/Marker.h>

#include <ros/ros.h>

class RobotController_Ackermann_Kinematic : public RobotController_Interpolation
{
public:
	RobotController_Ackermann_Kinematic(PathFollower* _path_follower);
	virtual ~RobotController_Ackermann_Kinematic(){}

	virtual void stopMotion();
	virtual void start();
	virtual bool isOmnidirectional() const {
		return false;
	}

protected:
	virtual MoveCommandStatus computeMoveCommand(MoveCommand* cmd);
	virtual void publishMoveCommand(const MoveCommand &cmd) const;

private:
	struct ControllerParameters : public RobotController_Interpolation::InterpolationParameters {
		P<double> vehicle_length;
		P<double> k_forward;
		P<double> k_backward;
		P<double> factor_k1;
		P<double> factor_k2;
		P<double> factor_k3;
		P<double> factor_steering_angle;
		P<double> max_steering_angle;
		P<double> max_steering_angle_speed;

		ControllerParameters() :
			vehicle_length(this, "~vehicle_length", 0.3, "axis-centre distance"),
			k_forward(this, "~k_forward", 7.0, "Tuning factor for forward driving"),
			k_backward(this, "~k_backward", 7.0, "Tuning factor for backward driving"),
			factor_k1(this, "~factor_k1", 1.0, "Factor for k1"),
			factor_k2(this, "~factor_k2", 3.0, "Factor for k2"),
			factor_k3(this, "~factor_k3", 3.0, "Factor for k3"),
			factor_steering_angle(this, "~factor_steering_angle", 1.0,
										 "Set 1.0 for one axis steering, 0.5 for two axis steering"),
			max_steering_angle(this, "~max_steering_angle", M_PI / 3, "Maximum steering angle"),
			max_steering_angle_speed(this, "~max_steering_angle_speed", 1.7, "Maximum steering angle speed")
		{}

	} params_;

	const RobotController_Interpolation::InterpolationParameters& getParameters() const
	{
		return params_;
	}

	void reset();

	//! Sets the tuning parameters k1, k2, k3 according to k
	void setTuningParameters(const double k);

	//! The MoveCommand that is beeing published
	MoveCommand move_cmd_;

	//! Tuning parameters
	double k1_, k2_, k3_;
	//! Last waypoint index
	unsigned int old_waypoint_;
	//! Current steering angle
	double phi_;
	//! v2 (steering angle speed)
	double v2_;
	//! ds/dt (path velocity of the last frame)
	double s_prim_;
	//! Last and before last steering angle
	double old_phi_, old_old_phi_;
	//! Last values of d and theta_e, used to compute derivations
	double old_d_, old_theta_e_;
	//! Derivations of d, theta_e, phi_prim, are only updated every few frames
	double d_prim_, theta_e_prim_, phi_prim_;
	//! The time of the last update (to compute the time that has passed since then)
	ros::Time old_time_;
};

#endif // ROBOTCONTROLLER_ACKERMANN_KINEMATIC_H
