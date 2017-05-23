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
    /**
     * @brief RobotController_2Steer_Stanley
     */
    RobotController_2Steer_Stanley();
    /**
     * @brief ~RobotController_2Steer_Stanley
     */
    virtual ~RobotController_2Steer_Stanley() {}
    /**
     * @brief stopMotion stops the robot
     */
	virtual void stopMotion();
    /**
     * @brief start
     */
	virtual void start();
    /**
     * @brief setPath sets the path
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
		P<double> vehicle_length;
		P<double> k_forward;
		P<double> k_backward;
		P<double> max_steering_angle;

		ControllerParameters() :
            RobotController::ControllerParameters("2steer_stanley"),

            vehicle_length(this, "vehicle_length", 0.34, "Axis-centre distance."),
            k_forward(this, "k_forward", 0.6, "Tuning factor for forward driving."),
            k_backward(this, "k_backward", 0.6, "Tuning factor for backward driving."),
            max_steering_angle(this, "max_steering_angle", 0.52359877559, "Maximum steering angle.")
		{}

	} params_;

    const RobotController::ControllerParameters& getParameters() const {
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
