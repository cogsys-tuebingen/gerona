#ifndef ROBOTCONTROLLER_4WS_PUREPURSUIT_H
#define ROBOTCONTROLLER_4WS_PUREPURSUIT_H

#include <path_follower/controller/robotcontroller_interpolation.h>
#include <path_follower/utils/parameters.h>


#define TEST_OUTPUT

#ifdef TEST_OUTPUT
#include <ros/ros.h>
#endif

class RobotController_4WS_PurePursuit: public RobotController_Interpolation
{
public:
	RobotController_4WS_PurePursuit(PathFollower* _path_follower);
	virtual ~RobotController_4WS_PurePursuit(){}

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

	struct ControllerParameters : public RobotController_Interpolation::InterpolationParameters {
		P<double> factor_lookahead_distance_forward;
		P<double> factor_lookahead_distance_backward;
		P<double> vehicle_length;

		ControllerParameters() :
			factor_lookahead_distance_forward(this, "~factor_lookahead_distance_forward", 0.8,
														 "lookahead distance factor while driving forwards"),
			factor_lookahead_distance_backward(this, "~factor_lookahead_distance_forward", 0.8,
														 "lookahead distance factor while driving backwards"),
			vehicle_length(this, "~vehicle_length", 0.34, "axis-centre distance")
		{}

	} params_;

	const RobotController_Interpolation::InterpolationParameters& getParameters() const {
		return params_;
	}

	double computeAlpha(double& lookahead_distance, const Eigen::Vector3d& pose);

	ros::NodeHandle node_handle_;
	ros::Publisher path_interpol_pub_;

	unsigned int waypoint_;
	MoveCommand move_cmd_;


#ifdef TEST_OUTPUT
	ros::Publisher test_pub_;
	void publishTestOutput(const unsigned int waypoint, const double d, const double theta_e,
								  const double phi, const double v) const;
#endif
};

#endif // ROBOTCONTROLLER_4WS_PUREPURSUIT_H
