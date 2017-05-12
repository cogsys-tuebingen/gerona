#ifndef ROBOTCONTROLLER_ACKERMANN_ORTHEXP_H
#define ROBOTCONTROLLER_ACKERMANN_ORTHEXP_H

/// PROJECT
#include <path_follower/controller/robotcontroller.h>
#include <path_follower/utils/parameters.h>
#include <path_follower/controller/robotcontroller_orthogonalexponential.h>

class RobotController_Ackermann_OrthogonalExponential: public RobotController_OrthogonalExponential
{
public:
    RobotController_Ackermann_OrthogonalExponential();

protected:
    virtual void computeControl();

private:
    struct ControllerParameters : public RobotController_OrthogonalExponential::ControllerParameters
    {
        P<double> k;

        ControllerParameters():
            RobotController_OrthogonalExponential::ControllerParameters("ackermann_orthexp"),

            k(this, "k", 1.5, "")
        {}
    } opt_;

    const RobotController::ControllerParameters& getParameters() const
    {
        return opt_;
    }
};

#endif // ROBOTCONTROLLER_ACKERMANN_ORTHEXP_H
