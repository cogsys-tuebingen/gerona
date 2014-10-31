#ifndef BEHAVIOURS_H
#define BEHAVIOURS_H

#include <ros/ros.h>

#include <path_follower/pathfollower.h>
#include <path_follower/utils/visualizer.h>
#include <utils_general/Line2d.h>


struct NullBehaviour {
};

class Behaviour
{
public:
    virtual ~Behaviour() {}

    virtual void execute(int* status) = 0;

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




struct BehaviourOnLine : public Behaviour
{
    BehaviourOnLine(PathFollower &parent);
    void execute(int *status);
    void getNextWaypoint();
};


struct BehaviourAvoidObstacle : public Behaviour
{
    BehaviourAvoidObstacle(PathFollower& parent)
        : Behaviour(parent)
    {}

    void execute(int *status);
    void getNextWaypoint();

    virtual bool isLeavingPathAllowed() const
    {
        return true;
    }
};

struct BehaviourApproachTurningPoint : public Behaviour
{
    BehaviourApproachTurningPoint(PathFollower &parent);

    void execute(int *status);

    bool checkIfDone();

    void handleDone();

    void getNextWaypoint();

    bool done_;
};


#endif // BEHAVIOURS_H
