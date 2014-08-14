#ifndef BEHAVIOURS_H
#define BEHAVIOURS_H

#include <ros/ros.h>

#include "pathfollower.h"
#include "visualizer.h"
#include <utils_general/Line2d.h>


struct NullBehaviour {
};

class Behaviour
{
protected:
    Behaviour(PathFollower& parent)
        : parent_(parent), controller_(parent.getController())
    {}
    Path& getSubPath(unsigned index);
    int getSubPathCount() const;
    PathFollower& getNode();

    PathFollower::Command& getCommand();//TODO: maybe Command is better placed in behaviours instead of PathFollower
    VectorFieldHistogram& getVFH();
    PathFollower::Options& getOptions();
    double distanceTo(const Waypoint& wp);

public:
    virtual ~Behaviour() {}

    virtual void execute(int* status) = 0;

protected:
    PathFollower& parent_;
    RobotController* controller_;
};


struct BehaviourEmergencyBreak : public Behaviour
{
    BehaviourEmergencyBreak(PathFollower& parent)
        : Behaviour(parent)
    {
        // stop immediately
        controller_->stopMotion();
    }

    void execute(int *status)
    {
        *status = path_msgs::FollowPathResult::MOTION_STATUS_INTERNAL_ERROR;
        throw new NullBehaviour;
    }
};


struct BehaviourDriveBase : public Behaviour
{
    BehaviourDriveBase(PathFollower &parent);

    //! Calculate the distance of the robot to the current path segment.
    double calculateDistanceToCurrentPathSegment();

    bool isCollision(double course);

    void setStatus(int status);//FIXME: is there a better solution than making this public? It should only be accessable for RobotController

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

    //! Pose of the next waypoint in map frame.
    geometry_msgs::PoseStamped next_wp_map_;

    //! Pose of the next waypoint in robot frame.
    Vector3d next_wp_local_;

    //! Timeout to abort, if the robot takes to long to reach the next waypoint.
    Timeout waypoint_timeout;

    Visualizer* visualizer_;

    void initExecute(int *status);

    //! Check if waypoint timeout has expired. If yes, switch to BehaviourEmergencyBreak.
    void checkWaypointTimeout();

    //! Check if the robot moves too far away from the path. If yes, switch to BehaviourEmergencyBreak.
    void checkDistanceToPath();

    PathWithPosition getPathWithPosition();

    virtual bool isLeavingPathAllowed() const
    {
        return false;
    }

    virtual void getNextWaypoint()
    {}
};



struct BehaviourOnLine : public BehaviourDriveBase
{
    BehaviourOnLine(PathFollower &parent);
    void execute(int *status);
    void getNextWaypoint();
};


struct BehaviourAvoidObstacle : public BehaviourDriveBase
{
    BehaviourAvoidObstacle(PathFollower& parent)
        : BehaviourDriveBase(parent)
    {}

    void execute(int *status);
    void getNextWaypoint();

    virtual bool isLeavingPathAllowed() const
    {
        return true;
    }
};

struct BehaviourApproachTurningPoint : public BehaviourDriveBase
{
    BehaviourApproachTurningPoint(PathFollower &parent);

    void execute(int *status);

    bool checkIfDone();

    void handleDone();

    void getNextWaypoint();

    bool done_;
};


#endif // BEHAVIOURS_H
