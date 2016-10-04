#ifndef ROBOTCONTROLLER_ACKERMANN_STANLEY_H
#define ROBOTCONTROLLER_ACKERMANN_STANLEY_H

#include <path_follower/controller/robotcontroller_interpolation.h>
#include <path_follower/utils/parameters.h>

class RobotController_Ackermann_Stanley : public RobotController_Interpolation
{
public:
	RobotController_Ackermann_Stanley(PathFollower* _path_follower);
	virtual ~RobotController_Ackermann_Stanley() {}

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
		P<double> factor_steering_angle;

		ControllerParameters() :
			vehicle_length(this, "~vehicle_length", 0.3, "axis-centre distance"),
			k_forward(this, "~k_forward", 7.0, "Tuning factor for forward driving"),
			k_backward(this, "~k_backward", 7.0, "Tuning factor for backward driving"),
			factor_steering_angle(this, "~factor_steering_angle", 1.0,
										 "Set 1.0 for one axis steering, 0.5 for two axis steering")
		{}

	} params_;

	const RobotController_Interpolation::InterpolationParameters& getParameters() const {
		return params_;
	}

	//! The MoveCommand that is beeing published
	MoveCommand move_cmd_;
};

#endif // ROBOTCONTROLLER_ACKERMANN_STANLEY_H
