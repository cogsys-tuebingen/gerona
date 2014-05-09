#ifndef ROBOTCONTROLLERACKERMANNPID_H
#define ROBOTCONTROLLERACKERMANNPID_H

/// ROS
#include <geometry_msgs/Twist.h>

/// THIRD PARTY
#include <Eigen/Core>

/// PROJECT
#include "robotcontroller.h"
#include "PidCtrl.h"
#include "vector_field_histogram.h"

class RobotController_Ackermann_Pid : public RobotController
{
public:
    RobotController_Ackermann_Pid(ros::Publisher &cmd_publisher,
                                  BehaviouralPathDriver *path_driver,
                                  VectorFieldHistogram *vfh);

    void configure();

    virtual bool setCommand(double error, double speed);

    virtual void publishCommand();

    virtual void stopMotion();

    virtual void initOnLine();
    virtual void behaveOnLine(PathWithPosition path);

    virtual void behaveAvoidObstacle(PathWithPosition path);

    virtual void initApproachTurningPoint();
    virtual bool behaveApproachTurningPoint(PathWithPosition path);

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

        bool isValid()
        {
            if ( isnan(velocity) || isinf(velocity)
                 || isnan(steer_front) || isinf(steer_front)
                 || isnan(steer_back) || isinf(steer_back) )
            {
                // fix this instantly, to avoid further problems.
                ROS_FATAL("Non-numerical values in command: %d,%d,%d,%d,%d,%d",
                          isnan(velocity), isinf(velocity),
                          isnan(steer_front), isinf(steer_front),
                          isnan(steer_back), isinf(steer_back));
                velocity = 0.0;
                steer_front = 0.0;
                steer_back = 0.0;

                return false;
            } else {
                return true;
            }
        }
    };

    struct ControllerOptions
    {
        double dead_time_;
        double l_;
    };


    PidCtrl pid_;
    Command cmd_;
    ControllerOptions options_;
    VectorFieldHistogram *vfh_;

    Eigen::Vector3d next_wp_local_;
    PathWithPosition path_;
    //! Step counter for behaviour ApproachTurningPoint.
    int atp_step_;

    inline void setStatus(int status);
    void setPath(PathWithPosition path);

    void predictPose(Eigen::Vector2d &front_pred, Eigen::Vector2d &rear_pred);
    double calculateCourse();
    //! Calculate the angle between the orientations of the waypoint and the robot.
    double calculateAngleError();
    double calculateLineError();
    double calculateDistanceError();
};

#endif // ROBOTCONTROLLERACKERMANNPID_H
