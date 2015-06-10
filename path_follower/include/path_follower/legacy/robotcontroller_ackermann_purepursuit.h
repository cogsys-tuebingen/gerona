/*
 * RobotcontrollerAckermannGeometrical.h
 *
 *  Created on: Apr 25, 2015
 *      Author: holly
 */

#ifndef NAVIGATION_PATH_FOLLOWER_INCLUDE_PATH_FOLLOWER_CONTROLLER_ROBOTCONTROLLER_ACKERMANN_GEOMETRICAL_H_
#define NAVIGATION_PATH_FOLLOWER_INCLUDE_PATH_FOLLOWER_CONTROLLER_ROBOTCONTROLLER_ACKERMANN_GEOMETRICAL_H_

#include <path_follower/controller/robotcontroller_interpolation.h>
#include <path_follower/utils/parameters.h>

#include <visualization_msgs/Marker.h>


class Robotcontroller_Ackermann_PurePursuit: public RobotController_Interpolation
{
public:
	Robotcontroller_Ackermann_PurePursuit(PathFollower* _path_follower);
	virtual ~Robotcontroller_Ackermann_PurePursuit();

	virtual void stopMotion();
	virtual void start();
	void reset();
	virtual bool isOmnidirectional() const {
		return false;
	}

protected:
	virtual MoveCommandStatus computeMoveCommand(MoveCommand* cmd);
	virtual void publishMoveCommand(const MoveCommand &cmd) const;

private:

	struct ControllerParameters : public RobotController_Interpolation::InterpolationParameters {
		P<double> factor_lookahead_distance;
		P<double> vehicle_length;
		P<double> factor_steering_angle;

		ControllerParameters() :
			factor_lookahead_distance(this, "~factor_lookahead_distance", 1.2, "lookahead distance factor"),
			vehicle_length(this, "~vehicle_length", 0.34, "axis-centre distance"),
			factor_steering_angle(this, "~factor_steering_angle", 1.0,
										 "Set 1.0 for one axis steering, 0.5 for two axis steering")
		{}

	} params;

	const RobotController_Interpolation::InterpolationParameters& getParameters() const {
		return params;
	}

	double computeAlpha(double& lookahead_distance, const Eigen::Vector3d& pose);

	ros::NodeHandle node_handle;
	ros::Publisher path_interpol_pub;

	unsigned int waypoint_;
	MoveCommand move_cmd;
};

#endif /* NAVIGATION_PATH_FOLLOWER_INCLUDE_PATH_FOLLOWER_CONTROLLER_ROBOTCONTROLLER_ACKERMANN_GEOMETRICAL_H_ */
