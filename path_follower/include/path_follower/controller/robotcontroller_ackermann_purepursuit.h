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
    /**
     * @brief Robotcontroller_Ackermann_PurePursuit
     */
    Robotcontroller_Ackermann_PurePursuit();
    /**
     * @brief ~Robotcontroller_Ackermann_PurePursuit
     */
	virtual ~Robotcontroller_Ackermann_PurePursuit();
    /**
     * @brief stopMotion stops the robot
     */
	virtual void stopMotion();
    /**
     * @brief start
     */
	virtual void start();
    /**
     * @brief reset
     */
    void reset();
    /**
     * @brief setPath sets the path
     */
	void setPath(Path::Ptr path);

protected:
    /**
     * @brief computeMoveCommand computes the command velocity for the robot
     *
     * The command velocity is computed for each controller differently. This is the core of
     * every controller. For more details, please visit: https://github.com/cogsys-tuebingen/gerona/wiki/controllers
     * On this wiki page, you will find references for each controller, where more mathematical and experimental details
     * can be found.
     *
     * @param cmd
     */
    virtual MoveCommandStatus computeMoveCommand(MoveCommand* cmd);
    /**
     * @brief publishMoveCommand publishes the computed move command
     *
     * The command velocity is set depending on the kinematics of the robot. E.g. for
     * differential drives the command input is (v, w), where v is linear, and w angular velocity,
     * and for an Ackermann drive, the command input is (v, phi), where v is linear velocity, and
     * phi is the steering angle. For an omnidirectional vehicle, it is possible to directly set
     * the linear velocity and the direction angle, while the rotation is set independently.
     *
     * @param cmd
     */
    virtual void publishMoveCommand(const MoveCommand &cmd) const;

private:

    struct ControllerParameters : public RobotController::ControllerParameters {
		P<double> factor_lookahead_distance_forward;
		P<double> factor_lookahead_distance_backward;
		P<double> vehicle_length;
		P<double> factor_steering_angle;

		ControllerParameters() :
            RobotController::ControllerParameters("ackermann_purepursuit"),

			factor_lookahead_distance_forward(this, "factor_lookahead_distance_forward", 0.8,
                                                         "Lookahead distance factor while driving forward."),
			factor_lookahead_distance_backward(this, "factor_lookahead_distance_forward", 0.8,
                                                         "Lookahead distance factor while driving backwards."),
            vehicle_length(this, "vehicle_length", 0.34, "Axis-centre distance."),
			factor_steering_angle(this, "factor_steering_angle", 1.0,
                                         "Set 1.0 for one axis steering, 0.5 for two axis steering.")
		{}

	} params_;

    const RobotController::ControllerParameters& getParameters() const {
		return params_;
	}

	double computeAlpha(double& lookahead_distance, const Eigen::Vector3d& pose);

	ros::NodeHandle node_handle_;
	ros::Publisher path_interpol_pub_;

	unsigned int waypoint_;
	MoveCommand move_cmd_;
};

#endif /* NAVIGATION_PATH_FOLLOWER_INCLUDE_PATH_FOLLOWER_CONTROLLER_ROBOTCONTROLLER_ACKERMANN_GEOMETRICAL_H_ */
