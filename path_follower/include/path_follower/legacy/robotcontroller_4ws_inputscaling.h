#ifndef ROBOTCONTROLLER_4WS_INPUTSCALING_H
#define ROBOTCONTROLLER_4WS_INPUTSCALING_H

#include <path_follower/controller/robotcontroller_interpolation.h>
#include <path_follower/utils/parameters.h>

#include <visualization_msgs/Marker.h>

#include <ros/ros.h>

#define TEST_OUTPUT

class RobotController_4WS_InputScaling : public RobotController_Interpolation
{
public:
	RobotController_4WS_InputScaling(PathFollower* _path_follower);
	virtual ~RobotController_4WS_InputScaling(){}

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
		P<double> max_steering_angle;
		P<double> max_steering_angle_speed;
		P<double> factor_velocity;

		ControllerParameters() :
			vehicle_length(this, "~vehicle_length", 0.3, "axis-centre distance"),
			k_forward(this, "~k_forward", 7.0, "Tuning factor for forward driving"),
			k_backward(this, "~k_backward", 7.0, "Tuning factor for backward driving"),
			factor_k1(this, "~factor_k1", 1.0, "Factor for k1"),
			factor_k2(this, "~factor_k2", 3.0, "Factor for k2"),
			factor_k3(this, "~factor_k3", 3.0, "Factor for k3"),
			max_steering_angle(this, "~max_steering_angle", M_PI / 3, "Maximum steering angle"),
			max_steering_angle_speed(this, "~max_steering_angle_speed", 1.7, "Maximum steering angle speed"),
			factor_velocity(this, "~factor_velocity", 5.0, "Factor that will be multiplied with the measured velocity")
		{}

	} params_;

	const RobotController_Interpolation::InterpolationParameters& getParameters() const
	{
		return params_;
	}

	void reset();
	void setPath(Path::Ptr path);

#ifdef TEST_OUTPUT
	ros::Publisher test_pub_;
	void publishTestOutput(const unsigned int waypoint, const double d, const double theta_e,
								  const double phi, const double v) const;
#endif

	//! Sets the tuning parameters k1, k2, k3 according to k
	void setTuningParameters(const double k);

	//! The MoveCommand that is beeing published
	MoveCommand move_cmd_;

	//! Tuning parameters
	double k1_, k2_, k3_;
	//! Current steering angle
	double phi_;
	//! v1 longitudinal velocity, v2 (steering angle speed)
	double v1_, v2_;
	//! ds/dt (path velocity of the last frame)
//	double s_prim_;
	//! The time of the last update (to compute the time that has passed since then)
	ros::Time old_time_;
};

#endif // ROBOTCONTROLLER_4WS_INPUTSCALING_H
