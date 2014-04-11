#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H

class BehaviouralPathDriver;

class RobotController
{
public:
    RobotController(BehaviouralPathDriver *path_driver) :
        path_driver_(path_driver)
    {}


    virtual void configure()
    {

    }

    //virtual bool setCommand(double error, double speed) = 0;

    //virtual void publishCommand() = 0;

    virtual void stopMotion() = 0;

    //virtual double calculateCourse() = 0;

    //virtual void predictPose() = 0; //TODO: arguments?


    /* BEHAVIOURS */
    virtual void behaveOnLine() = 0;
    virtual void behaveAvoidObstacle() = 0;
    virtual void behaveApproachTurningPoint() = 0;
    virtual void behaveEmergencyBreak() = 0;

protected:
    BehaviouralPathDriver *path_driver_;

};

#endif // ROBOTCONTROLLER_H
