#include <path_follower/supervisor/waypointtimeout.h>
#include <path_msgs/FollowPathResult.h>

WaypointTimeout::WaypointTimeout(ros::Duration max_duration):
    Timeout(max_duration)
{}


void WaypointTimeout::supervise(Supervisor::State &state, Supervisor::Result *out)
{
    out->can_continue = !isExpired();

    if (!out->can_continue) {
        ROS_WARN_NAMED("s_waypoint_timeout", "Waypoint Timeout! The robot did not reach the"
                       " next waypoint within %g sec. Abort path execution.",
                 duration_.toSec());
    }

    out->status = path_msgs::FollowPathResult::MOTION_STATUS_TIMEOUT;
}

void WaypointTimeout::eventNewGoal()
{
    reset();
}

void WaypointTimeout::eventNewWaypoint()
{
    reset();
}
