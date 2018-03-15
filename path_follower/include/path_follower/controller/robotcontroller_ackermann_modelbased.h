#ifndef ROBOTCONTROLLER_ACKERMANN_MODELBASED_H
#define ROBOTCONTROLLER_ACKERMANN_MODELBASED_H


/// PROJECT
#include <path_follower/controller/robotcontroller_modelbased.h>

/// The RobotController_Ackermann_ModelBased class
class RobotController_Ackermann_ModelBased: public RobotController_ModelBased
{
public:
    /**
     * @brief RobotController_Ackermann_ModelBased
     */
    RobotController_Ackermann_ModelBased();

protected:

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

    /**
     * @brief initialize
     */
    virtual void initialize();


    double wheelBaseLength_;
};



#endif // ROBOTCONTROLLER_POTENTIAL_FIELD_H
