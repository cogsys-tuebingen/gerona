#ifndef ROBOTCONTROLLER_OMNIDRIVE_ORTHEXP_H
#define ROBOTCONTROLLER_OMNIDRIVE_ORTHEXP_H

/// PROJECT
#include <path_follower/controller/robotcontroller.h>
#include <path_follower/utils/parameters.h>
#include <path_follower/controller/robotcontroller_orthogonalexponential.h>

class RobotController_Omnidrive_OrthogonalExponential: public RobotController_OrthogonalExponential
{
public:
    /**
     * @brief RobotController_Omnidrive_OrthogonalExponential
     */
    RobotController_Omnidrive_OrthogonalExponential();

protected:
    /**
     * @brief computeControl computes the command velocity specific for every orthogonal-exponential controller
     *
     * Orthogonal-exponential controller is in principle the same for every wheeled robot, but the command
     * output is computed differently for different kinematic types.
     *
     */
    virtual void computeControl();

private:
    struct ControllerParameters : public RobotController_OrthogonalExponential::ControllerParameters
    {
        P<double> k;
        P<double> kp;
        P<double> kd;

        ControllerParameters():
            RobotController_OrthogonalExponential::ControllerParameters("omnidrive_orthexp"),

            k(this, "k", 1.5, "Factor for regulating the convergence to the path."),
            kp(this, "kp", 0.4, "P component of the rotation control."),
            kd(this, "kd", 0.2, "D component of the rotation control.")
        {}
    } opt_;

    const RobotController::ControllerParameters& getParameters() const
    {
        return opt_;
    }

    double e_theta_curr_;
};

#endif // ROBOTCONTROLLER_OMNIDRIVE_ORTHEXP_H
