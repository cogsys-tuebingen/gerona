#ifndef ROBOTCONTROLLER_DIFFERENTIAL_ORTHEXP_H
#define ROBOTCONTROLLER_DIFFERENTIAL_ORTHEXP_H

/// PROJECT
#include <path_follower/controller/robotcontroller.h>
#include <path_follower/utils/parameters.h>
#include <path_follower/controller/robotcontroller_orthogonalexponential.h>

class RobotController_Differential_OrthogonalExponential: public RobotController_OrthogonalExponential
{
public:
    RobotController_Differential_OrthogonalExponential();


protected:
    virtual void computeControl();
    virtual void publishMoveCommand(const MoveCommand &cmd) const;

private:
    struct ControllerParameters : public RobotController_OrthogonalExponential::ControllerParameters
    {
        P<double> k;

        ControllerParameters():
            RobotController_OrthogonalExponential::ControllerParameters("differential_orthexp"),
            k(this, "k", 1.5, "")
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

