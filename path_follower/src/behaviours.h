#ifndef BEHAVIOURS_H
#define BEHAVIOURS_H

#include "BehaviouralPathDriver.h"
#include <utils_general/Line2d.h>


struct BehaviourEmergencyBreak : public motion_control::BehaviouralPathDriver::Behaviour
{
    BehaviourEmergencyBreak(motion_control::BehaviouralPathDriver& parent)
        : Behaviour(parent)
    {
        // stop immediately
        getCommand().velocity = 0;
        parent_.publishCommand();
    }

    void execute(int *status)
    {
        *status = path_msgs::FollowPathResult::MOTION_STATUS_INTERNAL_ERROR;
        throw new motion_control::BehaviouralPathDriver::NullBehaviour;
    }
};



struct BehaviourDriveBase : public motion_control::BehaviouralPathDriver::Behaviour
{
    BehaviourDriveBase(motion_control::BehaviouralPathDriver& parent)
        : Behaviour(parent)
    {}

    void getSlamPose();

    //! Calculate the angle between the orientations of the waypoint and the robot.
    double calculateAngleError();

    double calculateLineError();

    //! Calculate the distance of the robot to the current path segment.
    double calculateDistanceToCurrentPathSegment();

    void visualizeCarrot(const Vector2d& carrot, int id, float r, float g, float b);

    void visualizeLine(const Line2d& line);

    void setCommand(double error, double speed);

    void drawSteeringArrow(int id, geometry_msgs::Pose steer_arrow, double angle, double r, double g, double b);

protected:
    int* status_ptr_;

    geometry_msgs::Pose slam_pose_msg_;
    geometry_msgs::PoseStamped next_wp_map_;

    Vector3d next_wp_local_;
    double dir_sign_;
};



struct BehaviourOnLine : public BehaviourDriveBase
{
    BehaviourOnLine(motion_control::BehaviouralPathDriver& parent)
        : BehaviourDriveBase(parent)
    {}

    void execute(int *status);
    void getNextWaypoint();
};



struct BehaviourApproachTurningPoint : public BehaviourDriveBase
{
    BehaviourApproachTurningPoint(motion_control::BehaviouralPathDriver& parent)
        : BehaviourDriveBase(parent)
    {}

    void execute(int *status);

    double calculateDistanceError();

    void checkIfDone();

    void getNextWaypoint();
};


#endif // BEHAVIOURS_H
