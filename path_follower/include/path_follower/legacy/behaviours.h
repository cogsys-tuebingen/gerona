#ifndef BEHAVIOURS_H
#define BEHAVIOURS_H

#include <ros/ros.h>

#include <path_follower/pathfollower.h>
#include <path_follower/utils/visualizer.h>
#include <utils_general/Line2d.h>

class Behaviour
{
public:
    virtual ~Behaviour() {}

    virtual Behaviour* execute(int* status) = 0;

    void setStatus(int status);

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


    PathFollower& parent_;
    RobotController* controller_;
    int* status_ptr_;

    //! Pose of the next waypoint in map frame.
    geometry_msgs::PoseStamped next_wp_map_;

    //! Pose of the next waypoint in robot frame.
    Vector3d next_wp_local_;

    //! Timeout to abort, if the robot takes to long to reach the next waypoint.
    Timeout waypoint_timeout;

    Visualizer* visualizer_;


    Behaviour(PathFollower& parent);

    Path& getSubPath(unsigned index);
    int getSubPathCount() const;
    VectorFieldHistogram& getVFH();
    PathFollower::Options& getOptions();
    PathFollower::PathIndex& getPathIndex();
    double distanceTo(const Waypoint& wp);

    //! Calculate the distance of the robot to the current path segment.
    double calculateDistanceToCurrentPathSegment();


    Behaviour *initExecute(int *status);

    PathWithPosition getPathWithPosition();

    virtual bool isLeavingPathAllowed() const
    {
        return false;
    }

    virtual Behaviour* selectNextWaypoint()
    {
        return this;
    }
};




struct BehaviourEmergencyBreak : public Behaviour
{
    BehaviourEmergencyBreak(PathFollower& parent)
        : Behaviour(parent)
    {
        // stop immediately
        controller_->stopMotion();
    }

    Behaviour* execute(int *status)
    {
        ROS_WARN("commencing emergency break");
        *status = path_msgs::FollowPathResult::MOTION_STATUS_INTERNAL_ERROR;
        return NULL;
    }
};




struct BehaviourOnLine : public Behaviour
{
    BehaviourOnLine(PathFollower &parent);
    Behaviour* execute(int *status);
    Behaviour *selectNextWaypoint();
};


struct BehaviourApproachTurningPoint : public Behaviour
{
    BehaviourApproachTurningPoint(PathFollower &parent);

    Behaviour* execute(int *status);

    bool checkIfDone();

    Behaviour *handleDone();

    Behaviour* selectNextWaypoint();

    bool done_;
};


#endif // BEHAVIOURS_H
