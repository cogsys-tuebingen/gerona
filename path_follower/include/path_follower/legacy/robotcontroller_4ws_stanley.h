#ifndef ROBOTCONTROLLER_4WS_STANLEY_H
#define ROBOTCONTROLLER_4WS_STANLEY_H

#include <path_follower/controller/robotcontroller_interpolation.h>
#include <path_follower/utils/parameters.h>


#define TEST_OUTPUT

#ifdef TEST_OUTPUT
#include <ros/ros.h>
#endif
class RobotController_4WS_Stanley : public RobotController_Interpolation
{
public:
	RobotController_4WS_Stanley(PathFollower* _path_follower);
	virtual ~RobotController_4WS_Stanley() {}

	virtual void stopMotion();
	virtual void start();
	void setPath(Path::Ptr path);
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

		ControllerParameters() :
			vehicle_length(this, "~vehicle_length", 0.3, "axis-centre distance"),
			k_forward(this, "~k_forward", 7.0, "Tuning factor for forward driving"),
			k_backward(this, "~k_backward", 7.0, "Tuning factor for backward driving")
		{}

	} params_;

	const RobotController_Interpolation::InterpolationParameters& getParameters() const {
		return params_;
	}

	//! The MoveCommand that is beeing published
	MoveCommand move_cmd_;

#ifdef TEST_OUTPUT
	ros::Publisher test_pub_;
	void publishTestOutput(const unsigned int waypoint, const double d, const double theta_e,
								  const double phi, const double v) const;
#endif
};

#endif // ROBOTCONTROLLER_4WS_STANLEY_H
