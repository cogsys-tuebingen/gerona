#ifndef ROBOTCONTROLLER_DIFFERENTIAL_ORTHEXP_H
#define ROBOTCONTROLLER_DIFFERENTIAL_ORTHEXP_H

/// PROJECT
#include <path_follower/controller/robotcontroller.h>
#include <path_follower/utils/parameters.h>
#include <path_follower/controller/robotcontroller_orthogonalexponential.h>

class RobotController_Differential_OrthogonalExponential: public RobotController_OrthogonalExponential
{
public:
    /**
     * @brief RobotController_Differential_OrthogonalExponential
     */
    RobotController_Differential_OrthogonalExponential();


protected:
    /**
     * @brief computeControl computes the command velocity specific for every orthogonal-exponential controller
     *
     * Orthogonal-exponential controller is in principle the same for every wheeled robot, but the command
     * output is computed differently for different kinematic types.
     *
     */
    virtual void computeControl();
    /**
     * @brief publishMoveCommand publishes the move command specific for differential drives
     */
    virtual void publishMoveCommand(const MoveCommand &cmd) const;

private:
    struct ControllerParameters : public RobotController_OrthogonalExponential::ControllerParameters
    {
        P<double> k;

        ControllerParameters():
            RobotController_OrthogonalExponential::ControllerParameters("differential_orthexp"),
            k(this, "k", 1.5, "Factor for regulating the convergence to the path.")
        {}
    } opt_;

    const RobotController::ControllerParameters& getParameters() const
    {
        return opt_;
    }

    double theta_des_;
    double Ts_;

    double alpha_e_;
};
#endif // ROBOTCONTROLLER_DIFFERENTIAL_ORTHEXP_H

