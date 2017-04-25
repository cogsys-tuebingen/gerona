#ifndef ROBOTCONTROLLER_2STEER_STANLEY_H
#define ROBOTCONTROLLER_2STEER_STANLEY_H

#include <path_follower/controller/robotcontroller.h>
#include <path_follower/utils/parameters.h>


#define TEST_OUTPUT

#ifdef TEST_OUTPUT
#include <ros/ros.h>
#endif
class RobotController_2Steer_Stanley: public RobotController
{
public:
    RobotController_2Steer_Stanley();
    virtual ~RobotController_2Steer_Stanley() {}

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
    struct ControllerParameters : public RobotController::InterpolationParameters {
		P<double> vehicle_length;
		P<double> k_forward;
		P<double> k_backward;
		P<double> max_steering_angle;

		ControllerParameters() :
			vehicle_length(this, "~vehicle_length", 0.34, "axis-centre distance"),
			k_forward(this, "~k_forward", 0.6, "Tuning factor for forward driving"),
			k_backward(this, "~k_backward", 0.6, "Tuning factor for backward driving"),
			max_steering_angle(this, "~max_steering_angle", 0.52359877559, "Maximum steering angle")
		{}

	} params_;

    const RobotController::InterpolationParameters& getParameters() const {
		return params_;
	}

	//! The MoveCommand that is beeing published
	MoveCommand move_cmd_;

#ifdef TEST_OUTPUT
	//! A publisher to publish on "/test_output"
	ros::Publisher test_pub_;
	//! Publishes waypoint, d, theta_e, phi and v on "/test_ouput"
	void publishTestOutput(const unsigned int waypoint, const double d, const double theta_e,
								  const double phi, const double v) const;
#endif
};

#endif // ROBOTCONTROLLER_2STEER_STANLEY_H
