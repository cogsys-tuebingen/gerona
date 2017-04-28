#ifndef ROBOTCONTROLLER_2STEER_PUREPURSUIT_H
#define ROBOTCONTROLLER_2STEER_PUREPURSUIT_H

#include <path_follower/utils/parameters.h>
#include <path_follower/controller/robotcontroller.h>

#define TEST_OUTPUT

#ifdef TEST_OUTPUT
#include <ros/ros.h>
#endif

class RobotController_2Steer_PurePursuit: public RobotController
{
public:
    RobotController_2Steer_PurePursuit();
    virtual ~RobotController_2Steer_PurePursuit(){}

	virtual void stopMotion();
	virtual void start();
	void reset();
	void setPath(Path::Ptr path);
	virtual bool isOmnidirectional() const {
		return false;
	}

protected:
	virtual MoveCommandStatus computeMoveCommand(MoveCommand* cmd);
	virtual void publishMoveCommand(const MoveCommand &cmd) const;

private:

    struct ControllerParameters : public RobotController::InterpolationParameters {
		P<double> k_forward;
		P<double> k_backward;
		P<double> vehicle_length;

		ControllerParameters() :
			k_forward(this, "k_forward", 1.2, "lookahead distance factor while driving forwards"),
			k_backward(this, "k_forward", 1.2, "lookahead distance factor while driving backwards"),
			vehicle_length(this, "vehicle_length", 0.34, "axis-centre distance")
		{}

	} params_;

    const RobotController::InterpolationParameters& getParameters() const {
		return params_;
	}

	//! Computes alpha for a specific lookahead distance and sets the lookahead distance to the
	//! closest possible value
	double computeAlpha(double& lookahead_distance, const Eigen::Vector3d& pose);

	//! Last waypoint
	unsigned int waypoint_;
	//! The move command published
	MoveCommand move_cmd_;


#ifdef TEST_OUTPUT
	//! A publisher to publish on "/test_output"
	ros::Publisher test_pub_;
	//! Publishes waypoint, d, theta_e, phi and v on "/test_ouput"
	void publishTestOutput(const unsigned int waypoint, const double d, const double theta_e,
								  const double phi, const double v) const;
#endif
};

#endif // ROBOTCONTROLLER_2STEER_PUREPURSUIT_H
