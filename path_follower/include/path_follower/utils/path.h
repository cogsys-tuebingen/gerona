#ifndef PATH_H
#define PATH_H
/**
 * @brief Some simple classes to represent a path.
 */

#include <boost/function.hpp>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <Eigen/Core>

//! Waypoints define the path
struct Waypoint
{
    Waypoint() {}
    Waypoint(const geometry_msgs::PoseStamped& ref)
    {
        x = ref.pose.position.x;
        y = ref.pose.position.y;
        orientation = tf::getYaw(ref.pose.orientation);
    }
    operator geometry_msgs::Pose() const
    {
        geometry_msgs::Pose result;
        result.position.x = x;
        result.position.y = y;
        result.orientation = tf::createQuaternionMsgFromYaw(orientation);
        return result;
    }

    operator Eigen::Vector2d() const
    {
        return Eigen::Vector2d(x,y);
    }

    double distanceTo(const Waypoint& other) const
    {
        double dx = other.x - x;
        double dy = other.y - y;
        return std::sqrt(dx*dx + dy*dy);
    }

    //! x-coordinate of the waypoint's position
    double x;
    //! y-coordinate of the waypoint's position
    double y;
    //! Orientation of the waypoint, represented as an angle ("theta")
    double orientation;
};


//! A path is sequence of waypoints.
typedef std::vector<Waypoint> SubPath;

/**
 * @brief Wraps the path and manages the current sub path and waypoint.
 *
 * The path consists of a list of so called sub pathes where each sub
 * path is a list of waypoints.
 * The path is processed waypoint by waypoint using switchToNextWaypoint.
 * If the end of an sub path is reached, the next sub path has to be
 * selected using switchToNextSubPath().
 */
class Path {
public:
    typedef boost::shared_ptr<Path> Ptr;
    typedef boost::shared_ptr<Path const> ConstPtr;
    typedef boost::function<void ()> NextWaypointCallback_t;


    Path():
        current_sub_path(path_.begin()),
        has_callback_(false)
    {}

    //! Clear path and reset current subpath/waypoint.
    void clear();

    //! Set a new path. Current sub path and waypoint are set to the
    //! beginning of the path
    void setPath(const std::vector<SubPath> &path);

    /**
     * @brief Register callback function for next waypoint event.
     *
     * If set, the specified callback function is called every time
     * the current waypoint changes.
     * There can only be one function at the same time. Thus, if the
     * method is called twice, the second callback will replace the
     * first.
     *
     * @param func
     */
    void registerNextWaypointCallback(NextWaypointCallback_t func);

    //! Switch to the first waypoint in the next sub path.
    void switchToNextSubPath();

    //! Switch to the next waypoint in the current sub path
    void switchToNextWaypoint();

    //! Switch immediately to the last waypoint in the current sub path
    /** @todo  is there a cleaner way than such a method? */
    void switchToLastWaypoint();


    //! Returns true, if the path is empty.
    bool empty() const;

    //! Get the total number of sub paths in the path.
    size_t subPathCount() const;

    //! Returns true if the end of the path is reached (= last waypoint
    //! of the last sub path).
    bool isDone() const;

    //! Returns true if the end of the current sub path is reached.
    /** @todo really necessary? maybe better switch to next automatically */
    bool isSubPathDone() const;

    //! Returns true, if the current waypoint is the last one of the
    //! current sub path.
    bool isLastWaypoint() const;

    //! Get the current sub path.
    const SubPath &getCurrentSubPath() const;

    /**
     * @brief Get a waypoint on the current sub path.
     * @param idx Index of the waypoint.
     * @return Waypoint.
     */
    const Waypoint &getWaypoint(size_t idx) const;

    //! Get the current waypoint.
    const Waypoint &getCurrentWaypoint() const;

    //! Get the last waypoint of the current sub path.
    const Waypoint &getLastWaypoint() const;

    //! Get the index of the current waypoint.
    size_t getWaypointIndex() const;

private:
    std::vector<SubPath> path_;
    std::vector<SubPath>::const_iterator current_sub_path;
    size_t next_waypoint_idx_; //TODO: maybe use here iterator like for path?

    NextWaypointCallback_t next_wp_callback_;
    bool has_callback_;

    inline void fireNextWaypointCallback() const;
};


#endif // PATH_H
