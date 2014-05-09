#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H

/// THIRD PARTY
#include <Eigen/Core>

/// PROJECT
#include "path.h"

class BehaviouralPathDriver;

class RobotController
{
public:
    RobotController(ros::Publisher &cmd_publisher, BehaviouralPathDriver *path_driver) :
        cmd_pub_(cmd_publisher),
        path_driver_(path_driver),
        velocity_(0.0f)
    {
    }


    virtual void publishCommand() = 0;

    //! Immediatley stop any motion.
    virtual void stopMotion() = 0;


    /* BEHAVIOURS */
    virtual void initOnLine() {}
    virtual void behaveOnLine(PathWithPosition) = 0;
    virtual void initAvoidObstacle() {}
    virtual void behaveAvoidObstacle(PathWithPosition path) = 0;
    virtual void initApproachTurningPoint() {}
    //! Return true, when turning point is reached.
    virtual bool behaveApproachTurningPoint(PathWithPosition path) = 0;
    virtual void behaveEmergencyBreak() = 0;

    virtual void setVelocity(float v)
    {
        velocity_ = v;
    }

    virtual void setDirSign(float s)
    {
        dir_sign_ = s;
    }

    virtual float getDirSign() const
    {
        return dir_sign_;
    }

protected:
    ros::Publisher& cmd_pub_;

    BehaviouralPathDriver *path_driver_;

    //! Desired velocity (defined by the action goal).
    float velocity_;

    float filtered_speed_;

    //! Indicates the direction of movement (>0 -> forward, <0 -> backward)
    float dir_sign_;

    //! Current subpath.
    PathWithPosition path_;
    //! The next waypoint in the robot frame (set by setPath).
    Eigen::Vector3d next_wp_local_;

    virtual void setFilteredSpeed( const float speed ) {
        filtered_speed_ = speed;
    }

    virtual float getFilteredSpeed() const {
        return filtered_speed_;
    }

    void setStatus(int status);
    void setPath(PathWithPosition path);

    //! Calculate the angle between the orientations of the waypoint and the robot.
    virtual double calculateAngleError();
};

#endif // ROBOTCONTROLLER_H
