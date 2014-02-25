#ifndef PLANNER_NODE_H
#define PLANNER_NODE_H

/// PROJECT
#include <utils_path/common/SimpleGridMap2d.h>
#include <utils_path/common/Pose2d.h>

/// SYSTEM
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

/**
 * @brief The Planner class is a base class for other planning algorithms
 */
class Planner
{
public:
    /**
     * @brief Planner
     */
    Planner();

    /**
     * @brief ~Planner
     */
    virtual ~Planner();

    /**
     * @brief updateMapCallback is called when using map topics
     * @param map
     */
    void updateMapCallback(const nav_msgs::OccupancyGridConstPtr &map);

    /**
     * @brief updateGoalCallback is called when a new goal state is requested
     * @param goal
     */
    void updateGoalCallback (const geometry_msgs::PoseStampedConstPtr &goal);


protected:
    /**
     * @brief updateMap hook for when the map changes
     * @param map
     */
    virtual void updateMap(const nav_msgs::OccupancyGrid &map);

    /**
     * @brief plan is the interface for implementation classes
     * @param goal the requested goal message
     * @param from_world start pose in world coordinates
     * @param to_world goal pose in world coordinates
     * @param from_map start pose in map coordinates
     * @param to_map goal pose in map coordinates
     */
    virtual void plan (const geometry_msgs::PoseStamped &goal,
                       const lib_path::Pose2d& from_world, const lib_path::Pose2d& to_world,
                       const lib_path::Pose2d& from_map, const lib_path::Pose2d& to_map) = 0;


    /**
     * @brief interpolatePath interpolates segments of the given path, if their length exeeds the given maximum distance
     * @param path path to interpolate
     * @param max_distance maximum segment length
     * @return interpolated path
     */
    nav_msgs::Path interpolatePath(const nav_msgs::Path& path, double max_distance);

    /**
     * @brief smoothPath smooths the given path
     * @param path path to smooth
     * @param weight_data weight for data integrity
     * @param weight_smooth weight for smoothness
     * @param tolerance iteration stopping criterium
     * @return smooted path
     */
    nav_msgs::Path smoothPath(const nav_msgs::Path& path, double weight_data, double weight_smooth, double tolerance = 0.000001);

    /**
     * @brief convert convert a ros pose to a lib_path::Pose
     * @param rhs ros pose
     * @return lib_path::Pose
     */
    lib_path::Pose2d convert (const geometry_msgs::PoseStamped& rhs);

    /**
     * @brief publish publishes the given path and a smoothed version of it
     * @param path path to publish
     */
    void publish(const nav_msgs::Path& path);

private:
    nav_msgs::Path smoothPathSegment(const nav_msgs::Path& path, double weight_data, double weight_smooth, double tolerance);

    void subdividePath(nav_msgs::Path& result, geometry_msgs::PoseStamped low, geometry_msgs::PoseStamped up, double max_distance);

protected:
    ros::NodeHandle nh;

    bool use_map_topic_;
    bool use_map_service_;

    ros::Subscriber goal_pose_sub;

    ros::Subscriber map_sub;
    ros::ServiceClient map_service_client;

    tf::TransformListener tfl;

    std::string base_frame_;

    lib_path::SimpleGridMap2d * map_info;

private:
    ros::Publisher path_publisher;
    ros::Publisher raw_path_publisher;
};

#endif // PLANNER_NODE_H
