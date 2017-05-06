#ifndef ROBOTCONTROLLER_UNICYCLE_INPUTSCALING_H
#define ROBOTCONTROLLER_UNICYCLE_INPUTSCALING_H

#include <path_follower/controller/robotcontroller.h>
#include <path_follower/utils/parameters.h>

#include <ros/ros.h>

#define TEST_OUTPUT

class RobotController_Unicycle_InputScaling: public RobotController
{
public:
    RobotController_Unicycle_InputScaling();
    virtual ~RobotController_Unicycle_InputScaling(){}

    virtual void stopMotion();
    virtual void start();
    virtual bool isOmnidirectional() const {
        return false;
    }

protected:
    virtual MoveCommandStatus computeMoveCommand(MoveCommand* cmd);
    virtual void publishMoveCommand(const MoveCommand &cmd) const;

private:
    struct ControllerParameters : public RobotController::ControllerParameters {
        P<double> vehicle_length;
        P<double> k;
        P<double> max_angular_velocity;

        ControllerParameters() :
            RobotController::ControllerParameters("unicycle_inputscaling"),

            vehicle_length(this, "vehicle_length", 0.3, "axis-centre distance"),
            k(this, "k", 7.0, "Tuning factor"),
            max_angular_velocity(this, "maximum_angular_velocity", 0.8, "Maximum angular velocity")
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
    void publishTestOutput(const unsigned int waypoint, const double d, const double theta_e, const double v) const;
#endif

    //! Sets the tuning parameters k1, k2, k3 according to k
    void setTuningParameters(const double k);

    //! Converts the steering angle to compensate the errors of the robot
    double lookUpAngle(const double angle) const;

    //! The MoveCommand that is beeing published
    MoveCommand move_cmd_ = true;

    //! Tuning parameters
    double k1_, k2_;

};

#endif // ROBOTCONTROLLER_UNICYCLE_INPUTSCALING_H
