/**
 * @file CombinedPlannerNode.h
 * @date Jan 2012
 * @author marks
 */

// C/C++
#include <string>
#include <vector>
#include <list>

// ROS
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>

// Workspace
#include <utils/LibPath/common/Point2d.h>
#include <utils/LibPath/common/SimpleGridMap2d.h>
#include <utils/LibRosUtil/Costmap2dWrapper.h>
#include <utils/LibRosUtil/OccupancyGridWrapper.h>

// Project
#include "GlobalPlanner.h"
#include "LocalPlanner.h"

class CombinedPlannerNode {
public:

    CombinedPlannerNode();

    void updateMap( const nav_msgs::OccupancyGridConstPtr& map );
    void updateGoal( const geometry_msgs::PoseStampedConstPtr& goal );
    void update( bool force_replan = false );
    bool getRobotPose( geometry_msgs::Pose& pose, const std::string& map_frame );
    void publishLocalPath( std::list<lib_path::Pose2d>& path );
    void publishEmptyLocalPath();
    void visualizePath( const std::vector<lib_path::Point2d> &path,
                        const std::string& ns,
                        const int color = 0,
                        const int id = 0 );

    void visualizeWaypoints( const std::list<lib_path::Pose2d> &wp, std::string ns, int id );

    void generateWaypoints( const std::vector<lib_path::Point2d> &path,
                            const lib_path::Pose2d& goal,
                            list<lib_path::Pose2d> &waypoints ) const;

    bool nextWaypoint( const lib_path::Pose2d& robot_pose ) const;
    bool isGoalReached( const lib_path::Pose2d& robot_pose, const lib_path::Pose2d& goal ) const;
    void getPoseDelta( const lib_path::Pose2d& p, const lib_path::Pose2d& q, double& d_dist, double& d_theta ) const;

    lib_path::Pose2d getNormalizedDelta( const lib_path::Point2d& start, const lib_path::Point2d& end ) const;

private:
    /// Name of the global map topic we are listening on
    std::string map_topic_;
    /// Name of the goal topic we are listening on
    std::string goal_topic_;
    /// Name of the topic to publish the path
    std::string path_topic_;

    // ROS stuff
    ros::NodeHandle n_;
    ros::Publisher path_pub_;
    ros::Publisher visu_pub_;
    ros::Subscriber map_subs_;
    ros::Subscriber goal_subs_;
    tf::TransformListener tf_;

    /// Used to build up a local costmap from laser data
    costmap_2d::Costmap2DROS lmap_ros_;

    /// It's seems to be impossible to work on the ROS costmap directly, so we have to copy the data
    costmap_2d::Costmap2D lmap_cpy_;

    /// Changes the interface of the local costmap
    lib_ros_util::Costmap2dWrapper lmap_wrapper_;

    /// Copy of the global map we are planning on
    nav_msgs::OccupancyGrid gmap_cpy_;

    /// Changes the interface of the global map
    lib_ros_util::OccupancyGridWrapper gmap_wrapper_;

    /// Frame id of the latest global map (usually "/map")
    std::string map_frame_id_;

    /// The planner used to calculate a global path
    GlobalPlanner* global_planner_;

    /// The local planner working on the local costmap
    LocalPlanner* local_planner_;

    /// True if we got at least one global map
    bool got_map_;

    /// The waypoints of the current path
    std::list<lib_path::Pose2d> waypoints_;
};
