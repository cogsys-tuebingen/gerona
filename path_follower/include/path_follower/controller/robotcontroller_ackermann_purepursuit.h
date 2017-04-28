/*
 * RobotcontrollerAckermannGeometrical.h
 *
 *  Created on: Apr 25, 2015
 *      Author: holly
 */

#ifndef NAVIGATION_PATH_FOLLOWER_INCLUDE_PATH_FOLLOWER_CONTROLLER_ROBOTCONTROLLER_ACKERMANN_GEOMETRICAL_H_
#define NAVIGATION_PATH_FOLLOWER_INCLUDE_PATH_FOLLOWER_CONTROLLER_ROBOTCONTROLLER_ACKERMANN_GEOMETRICAL_H_

#include <path_follower/utils/parameters.h>
#include <path_follower/controller/robotcontroller.h>

class Robotcontroller_Ackermann_PurePursuit: public RobotController
{
public:
    Robotcontroller_Ackermann_PurePursuit();
	virtual ~Robotcontroller_Ackermann_PurePursuit();

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
		P<double> factor_lookahead_distance_forward;
		P<double> factor_lookahead_distance_backward;
		P<double> vehicle_length;
		P<double> factor_steering_angle;

		ControllerParameters() :
			factor_lookahead_distance_forward(this, "factor_lookahead_distance_forward", 0.8,
														 "lookahead distance factor while driving forwards"),
			factor_lookahead_distance_backward(this, "factor_lookahead_distance_forward", 0.8,
														 "lookahead distance factor while driving backwards"),
			vehicle_length(this, "vehicle_length", 0.34, "axis-centre distance"),
			factor_steering_angle(this, "factor_steering_angle", 1.0,
										 "Set 1.0 for one axis steering, 0.5 for two axis steering")
		{}

	} params_;

    const RobotController::InterpolationParameters& getParameters() const {
		return params_;
	}

	double computeAlpha(double& lookahead_distance, const Eigen::Vector3d& pose);

	ros::NodeHandle node_handle_;
	ros::Publisher path_interpol_pub_;

	unsigned int waypoint_;
	MoveCommand move_cmd_;
};

#endif /* NAVIGATION_PATH_FOLLOWER_INCLUDE_PATH_FOLLOWER_CONTROLLER_ROBOTCONTROLLER_ACKERMANN_GEOMETRICAL_H_ */
