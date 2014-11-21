#ifndef WAYPOINTTIMEOUT_H
#define WAYPOINTTIMEOUT_H

#include <path_follower/supervisor/supervisor.h>
#include <ros/time.h>


//! Very simple timeout class.
class Timeout {
public:
    Timeout(ros::Duration max_duration):
        duration_(max_duration)
    {
        reset();
    }

    ros::Duration duration_;

    void reset() {
        started_ = ros::Time::now();
    }

    bool isExpired() {
        return (started_ + duration_) < ros::Time::now();
    }

private:
    ros::Time started_;
};

//! This supervisor tells the robot to stop if the time to reach the
//! next waypoint exceeds the specified timeout.
class WaypointTimeout : public Supervisor, private Timeout
{
public:
    WaypointTimeout(ros::Duration max_duration);

    virtual void supervise(Supervisor::State &state, Supervisor::Result *out);
    virtual void eventNewGoal();
    virtual void eventNewWaypoint();
};

#endif // WAYPOINTTIMEOUT_H
