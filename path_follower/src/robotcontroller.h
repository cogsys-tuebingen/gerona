#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H

#include "path.h"
#include "behaviours.h"

class BehaviouralPathDriver;

class RobotController
{
public:
    RobotController(BehaviouralPathDriver *path_driver) :
        path_driver_(path_driver),
        velocity_(0.0f)
    {
        configure();
    }


    virtual void configure()
    {

    }

    //virtual bool setCommand(double error, double speed) = 0;

    //virtual void publishCommand() = 0;

    virtual void stopMotion() = 0;

    //virtual double calculateCourse() = 0;

    //virtual void predictPose() = 0; //TODO: arguments?


    /* BEHAVIOURS */
    virtual void behaveOnLine(PathWithPosition) = 0;
    virtual void behaveAvoidObstacle(PathWithPosition path) = 0;
    virtual void behaveApproachTurningPoint(PathWithPosition path) = 0;
    virtual void behaveEmergencyBreak() = 0;

    virtual void setVelocity(float v)
    {
        velocity_ = v;
    }

protected:
    BehaviouralPathDriver *path_driver_;

    //! Desired velocity (defined by the action goal).
    float velocity_;

};

#endif // ROBOTCONTROLLER_H
