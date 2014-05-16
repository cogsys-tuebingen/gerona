#ifndef ROBOTCONTROLLER_OMNIDRIVE_PID_H
#define ROBOTCONTROLLER_OMNIDRIVE_PID_H

/// PROJECT
#include "robotcontroller.h"
#include "PidCtrl.h"

class RobotController_Omnidrive_Pid : public RobotController
{
public:
    RobotController_Omnidrive_Pid(ros::Publisher &cmd_publisher, BehaviouralPathDriver *path_driver);

    virtual void publishCommand() = 0;

    //! Immediatley stop any motion.
    virtual void stopMotion() = 0;


    /* BEHAVIOURS */

    virtual void initOnLine();


protected:
    virtual void behaveOnLine();

    virtual void behaveAvoidObstacle() = 0;

    //! Return true, when turning point is reached.
    virtual bool behaveApproachTurningPoint() = 0;


private:
    struct Command
    {
        float velocity;
        //double steer_front;

        // initialize all values to zero
        Command():
            velocity(0.0f)
        {}

//        operator geometry_msgs::Twist()
//        {
//            geometry_msgs::Twist msg;
//            msg.linear.x  = velocity;
//            msg.angular.z = steer_front;
//            return msg;
//        }

//        bool isValid()
//        {
//            if ( isnan(velocity) || isinf(velocity)
//                 || isnan(steer_front) || isinf(steer_front)
//                 || isnan(steer_back) || isinf(steer_back) )
//            {
//                // fix this instantly, to avoid further problems.
//                ROS_FATAL("Non-numerical values in command: %d,%d,%d,%d,%d,%d",
//                          isnan(velocity), isinf(velocity),
//                          isnan(steer_front), isinf(steer_front),
//                          isnan(steer_back), isinf(steer_back));
//                velocity = 0.0;
//                steer_front = 0.0;
//                steer_back = 0.0;

//                return false;
//            } else {
//                return true;
//            }
//        }
    };



    PidCtrl pid_;
    Command cmd_;

    void configure();
};

#endif // ROBOTCONTROLLER_OMNIDRIVE_PID_H
