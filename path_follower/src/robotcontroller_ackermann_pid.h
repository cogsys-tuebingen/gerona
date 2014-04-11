#ifndef ROBOTCONTROLLERACKERMANNPID_H
#define ROBOTCONTROLLERACKERMANNPID_H

/// ROS
#include <geometry_msgs/Twist.h>

/// PROJECT
#include "robotcontroller.h"
#include "PidCtrl.h"

class RobotController_Ackermann_Pid : public RobotController
{
public:
    RobotController_Ackermann_Pid(BehaviouralPathDriver *path_driver);

    virtual void configure();

    virtual bool setCommand(double error, double speed);

    virtual void publishCommand();

    virtual void stopMotion();

    virtual void behaveOnLine();
    virtual void behaveAvoidObstacle();
    virtual void behaveApproachTurningPoint();
    virtual void behaveEmergencyBreak();

private:
    struct Command
    {
        double velocity;
        double steer_front;
        double steer_back;

        /* ramaxx_msg commented for the moment, as it would limit the use of this node to the rabots.
         * Reimplement this, if you want to use a robot with front and back steerting.
         */
        /*
        operator ramaxx_msgs::RamaxxMsg()
        {
            ramaxx_msgs::RamaxxMsg msg;
            msg.data.resize(3);
            msg.data[0].key = ramaxx_msgs::RamaxxMsg::CMD_STEER_FRONT_DEG;
            msg.data[1].key = ramaxx_msgs::RamaxxMsg::CMD_STEER_REAR_DEG;
            msg.data[2].key = ramaxx_msgs::RamaxxMsg::CMD_SPEED;
            msg.data[0].value = steer_front * 180.0/M_PI;
            msg.data[1].value = steer_back * 180.0/M_PI;
            msg.data[2].value = v;
            return msg;
        }
        */

        operator geometry_msgs::Twist()
        {
            geometry_msgs::Twist msg;
            msg.linear.x  = velocity;
            msg.angular.z = steer_front;
            return msg;
        }
    };


    PidCtrl pid_;
    Command cmd_;


    double calculateAngleError();
    double calculateLineError();
};

#endif // ROBOTCONTROLLERACKERMANNPID_H
