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
    /**
     * @brief RobotController_2Steer_PurePursuit
     */
    RobotController_2Steer_PurePursuit();
    /**
     * @brief ~RobotController_2Steer_PurePursuit
     */
    virtual ~RobotController_2Steer_PurePursuit(){}
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
     * @brief setPath sets new path
     * @param path
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
		P<double> k_forward;
		P<double> k_backward;
		P<double> vehicle_length;

		ControllerParameters() :
            RobotController::ControllerParameters("2steer_purepursuit"),

            k_forward(this, "k_forward", 1.2, "Look-ahead distance factor while driving forwards."),
            k_backward(this, "k_forward", 1.2, "Look-ahead distance factor while driving backwards."),
            vehicle_length(this, "vehicle_length", 0.34, "Axis-centre distance.")
		{}

	} params_;

    const RobotController::ControllerParameters& getParameters() const {
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
