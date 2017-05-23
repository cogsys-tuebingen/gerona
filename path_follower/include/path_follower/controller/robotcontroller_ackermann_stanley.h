#ifndef ROBOTCONTROLLER_ACKERMANN_STANLEY_H
#define ROBOTCONTROLLER_ACKERMANN_STANLEY_H

#include <path_follower/controller/robotcontroller.h>
#include <path_follower/utils/parameters.h>

class RobotController_Ackermann_Stanley: public RobotController
{
public:
    /**
     * @brief RobotController_Ackermann_Stanley
     */
	RobotController_Ackermann_Stanley();
    /**
     * @brief ~RobotController_Ackermann_Stanley
     */
	virtual ~RobotController_Ackermann_Stanley() {}
    /**
     * @brief stopMotion stops the robot
     */
	virtual void stopMotion();
    /**
     * @brief start
     */
	virtual void start();

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
		P<double> factor_steering_angle;

		ControllerParameters() :
            RobotController::ControllerParameters("ackermann_stanley"),

            vehicle_length(this, "vehicle_length", 0.3, "Axis-centre distance."),
            k_forward(this, "k_forward", 7.0, "Tuning factor for forward driving."),
            k_backward(this, "k_backward", 7.0, "Tuning factor for backward driving."),
			factor_steering_angle(this, "factor_steering_angle", 1.0,
                                         "Set 1.0 for one axis steering, 0.5 for two axis steering.")
		{}

	} params_;

    const RobotController::ControllerParameters& getParameters() const {
		return params_;
	}

    void reset();
    void setPath(Path::Ptr path);

	//! The MoveCommand that is beeing published
	MoveCommand move_cmd_;
};

#endif // ROBOTCONTROLLER_ACKERMANN_STANLEY_H
