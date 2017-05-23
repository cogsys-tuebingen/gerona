#ifndef ROBOTCONTROLLER_2STEER_INPUTSCALING_H
#define ROBOTCONTROLLER_2STEER_INPUTSCALING_H

#include <path_follower/utils/parameters.h>
#include <path_follower/controller/robotcontroller.h>

#include <ros/ros.h>

#define TEST_OUTPUT

class RobotController_2Steer_InputScaling : public RobotController
{
public:
    /**
     * @brief RobotController_2Steer_InputScaling
     */
    RobotController_2Steer_InputScaling();
    /**
     * @brief ~RobotController_2Steer_InputScaling
     */
    virtual ~RobotController_2Steer_InputScaling(){}
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
		P<double> max_steering_angle;
		P<double> max_steering_angle_speed;

		ControllerParameters() :
            RobotController::ControllerParameters("2steer_inputscaling"),

            vehicle_length(this, "vehicle_length", 0.3, "Axis-centre distance."),
            k_forward(this, "k_forward", 7.0, "Tuning factor for forward driving."),
            k_backward(this, "k_backward", 7.0, "Tuning factor for backward driving."),
            max_steering_angle(this, "max_steering_angle", M_PI / 3.0, "Maximum steering angle."),
            max_steering_angle_speed(this, "max_steering_angle_speed", 1.7, "Maximum steering angle speed.")
		{}

	} params_;

    const RobotController::ControllerParameters& getParameters() const {
		return params_;
	}

	void reset();
	void setPath(Path::Ptr path);

#ifdef TEST_OUTPUT
	//! A publisher to publish on "/test_output"
	ros::Publisher test_pub_;
	//! Publishes waypoint, d, theta_e, phi and v on "/test_ouput"
	void publishTestOutput(const unsigned int waypoint, const double d, const double theta_e,
								  const double phi, const double v) const;
#endif

	//! Sets the tuning parameters k1, k2, k3 according to k
    void setTuningParameters(const double k);

	//! The MoveCommand that is beeing published
	MoveCommand move_cmd_;

	//! Tuning parameters
	double k1_, k2_, k3_;
	//! Current steering angle
	double phi_;
	//! The time of the last update (to compute the time that has passed since then)
	ros::Time old_time_;
};

#endif // ROBOTCONTROLLER_2STEER_INPUTSCALING_H
