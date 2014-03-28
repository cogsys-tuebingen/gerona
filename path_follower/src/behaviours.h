#ifndef BEHAVIOURS_H
#define BEHAVIOURS_H

#include <ros/ros.h>

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
    BehaviourDriveBase(motion_control::BehaviouralPathDriver& parent);

    void getSlamPose();

    //! Calculate the angle between the orientations of the waypoint and the robot.
    double calculateAngleError();

    double calculateLineError();

    //! Calculate the distance of the robot to the current path segment.
    double calculateDistanceToCurrentPathSegment();

    void visualizeCarrot(const Vector2d& carrot, int id, float r, float g, float b);

    void visualizeLine(const Line2d& line);

    double calculateCourse();
    bool isCollision(double course);
    bool setCommand(double error, double speed);

    void drawSteeringArrow(int id, geometry_msgs::Pose steer_arrow, double angle, double r, double g, double b);

protected:
    //! Very simple timeout class.
    class Timeout {
    public:
        Timeout() {
            reset();
        }

        ros::Duration duration;

        void reset() {
            started = ros::Time::now();
        }

        bool isExpired() {
            return (started + duration) < ros::Time::now();
        }

    private:
        ros::Time started;
    };



    int* status_ptr_;

    //! Pose of the robot.
    geometry_msgs::Pose slam_pose_msg_;

    //! Pose of the next waypoint in map frame.
    geometry_msgs::PoseStamped next_wp_map_;

    //! Pose of the next waypoint in robot frame.
    Vector3d next_wp_local_;

    //! Indicates the direction of movement (>0 -> forward, <0 -> backward)
    double dir_sign_;

    //! Timeout to abort, if the robot takes to long to reach the next waypoint.
    Timeout waypoint_timeout;

    //! Check if waypoint timeout has expired. If yes, switch to BehaviourEmergencyBreak.
    void checkWaypointTimeout();
};



struct BehaviourOnLine : public BehaviourDriveBase
{
    BehaviourOnLine(motion_control::BehaviouralPathDriver& parent);
    void execute(int *status);
    void getNextWaypoint();
};


struct BehaviourAvoidObstacle : public BehaviourDriveBase
{
    BehaviourAvoidObstacle(motion_control::BehaviouralPathDriver& parent)
        : BehaviourDriveBase(parent)
    {}

    void execute(int *status);
    void getNextWaypoint();
};

struct BehaviourApproachTurningPoint : public BehaviourDriveBase
{
    BehaviourApproachTurningPoint(motion_control::BehaviouralPathDriver& parent);

    void execute(int *status);

    double calculateDistanceError();

    bool checkIfDone(bool done = false);

    void getNextWaypoint();

    int step;
    bool waiting_;
};


#endif // BEHAVIOURS_H
