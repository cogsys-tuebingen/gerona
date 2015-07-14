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
		P<double> factor_steering_angle_forward;
		P<double> factor_steering_angle_backward;
		P<double> max_steering_angle;
		P<double> max_steering_angle_speed;

		ControllerParameters() :
			vehicle_length(this, "~vehicle_length", 0.3, "axis-centre distance"),
			k_forward(this, "~k_forward", 7.0, "Tuning factor for forward driving"),
			k_backward(this, "~k_backward", 7.0, "Tuning factor for backward driving"),
			factor_steering_angle_forward(this, "~factor_steering_angle_forward", 1.0,
										 "Set 1.0 for one axis steering, 0.5 for two axis steering"),
			factor_steering_angle_backward(this, "~factor_steering_angle_backward", 1.0,
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
	void setTuningParameters(const double k);

	ros::NodeHandle node_handle_;
	ros::Publisher path_interpol_pub_;

	MoveCommand move_cmd_;

	double k1_, k2_, k3_;
	double delta_;
	ros::Time old_time_;
};

#endif // ROBOTCONTROLLER_ACKERMANN_KINEMATIC_H
