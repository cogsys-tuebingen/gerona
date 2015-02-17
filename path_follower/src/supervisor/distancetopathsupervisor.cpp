#include <path_follower/supervisor/distancetopathsupervisor.h>
#include <path_msgs/FollowPathResult.h>
#include <paths.h>
#include <utils_general/Line2d.h>
#include <Eigen/Core>

using namespace Eigen;

namespace {
//! Module name, that is used for ros console output
const std::string MODULE = "s_disttopath";
}

DistanceToPathSupervisor::DistanceToPathSupervisor(double max_distance_to_path):
    max_dist_(max_distance_to_path),
    visualizer_(Visualizer::getInstance())
{}

void DistanceToPathSupervisor::supervise(Supervisor::State &state, Supervisor::Result *out)
{
    double dist = calculateDistanceToCurrentPathSegment(state);
    ROS_DEBUG_NAMED(MODULE, "Distance to current path segment: %g m", dist);
    if (dist > max_dist_) {
        //parent_.say("abort: too far away!"); //TODO: give supervisors access to say().

        ROS_WARN_NAMED(MODULE, "Moved too far away from the path (%g m, limit: %g m). Abort.",
                 dist, max_dist_);

        out->can_continue = false;
        out->status = path_msgs::FollowPathResult::RESULT_STATUS_PATH_LOST;
    }
}

double DistanceToPathSupervisor::calculateDistanceToCurrentPathSegment(const Supervisor::State &state)
{
    /* Calculate line from last way point to current way point (which should be the line the robot is driving on)
     * and calculate the distance of the robot to this line.
     */

    // Get previous waypoint
    // If the current waypoint is the first in this sub path (i.e. index == 0), use the next
    // instead. (I am not absolutly sure if this a good behaviour, so observe this via debug-output).
    int wp1_idx = 0;
    if (state.path->getWaypointIndex() > 0) {
        wp1_idx = state.path->getWaypointIndex() - 1;
    } else {
        // if wp_idx == 0, use the segment from 0th to 1st waypoint.
        wp1_idx = 1;

        ROS_DEBUG_NAMED(MODULE, "Toggle waypoints as wp_idx == 0 in calculateDistanceToCurrentPathSegment() (%s, line %d)", __FILE__, __LINE__);
    }

    geometry_msgs::Pose wp1 = state.path->getWaypoint(wp1_idx);
    geometry_msgs::Pose wp2 = state.path->getCurrentWaypoint();

    // line from last waypoint to current one.
    Line2d segment_line(Vector2d(wp1.position.x, wp1.position.y), Vector2d(wp2.position.x, wp2.position.y));

    ///// visualize start and end point of the current segment (for debugging)
    visualizer_->drawMark(24, wp1.position, "segment_marker", 0, 1, 1);
    visualizer_->drawMark(25, wp2.position, "segment_marker", 1, 0, 1);
    /////

    // get distance of robot (slam_pose_) to segment_line.
    return segment_line.GetDistance(state.robot_pose.head<2>());
}
