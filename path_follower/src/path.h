#ifndef PATH_H
#define PATH_H
/**
 * @brief Some simple classes to represent a path.
 */

#include <geometry_msgs/Pose.h>
#include <tf/tf.h>

//! Waypoints define the path
struct Waypoint
{
    Waypoint() {}
    Waypoint(const geometry_msgs::PoseStamped& ref)
    {
        x = ref.pose.position.x;
        y = ref.pose.position.y;
        theta = tf::getYaw(ref.pose.orientation);
    }
    operator geometry_msgs::Pose() const
    {
        geometry_msgs::Pose result;
        result.position.x = x;
        result.position.y = y;
        result.orientation = tf::createQuaternionMsgFromYaw(theta);
        return result;
    }

    double distanceTo(const Waypoint& other) const
    {
        double dx = other.x - x;
        double dy = other.y - y;
        return std::sqrt(dx*dx + dy*dy);
    }

    double x;
    double y;
    double theta;
};


//! A path is sequence of waypoints.
typedef std::vector<Waypoint> Path;


//! A simple wrapper for the current path and the id of the next waypoint on this path.
struct PathWithPosition {
    //! The current sub path
    Path *current_path;
    //! Index of the next waypoint
    size_t wp_idx;

    PathWithPosition() {
        current_path = 0;
    };

    PathWithPosition(Path *p, size_t wpid) :
        current_path(p),
        wp_idx(wpid)
    {}

    Waypoint getWaypoint(size_t idx)
    {
        return (*current_path)[idx];
    }

    Waypoint nextWaypoint()
    {
        return (*current_path)[wp_idx];
    }
};

#endif // PATH_H
