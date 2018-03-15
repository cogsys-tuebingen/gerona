#ifndef PLANNER_NODE_H
#define PLANNER_NODE_H

/// PROJECT
#include <cslibs_path_planning/common/SimpleGridMap2d.h>
#include <cslibs_path_planning/common/Pose2d.h>
#include <path_msgs/PlanPathAction.h>

/// SYSTEM
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/core/core.hpp>

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
    virtual void updateMap(const nav_msgs::OccupancyGrid &map, bool is_cost_map);

    /**
     * @brief execute ActionLib interface
     * @param goal
     */
    virtual void execute(const path_msgs::PlanPathGoalConstPtr &goal);

    /**
     * @brief plan is the interface for implementation classes
     * @param goal the requested goal message
     */
    path_msgs::PathSequence planImpl (const path_msgs::PlanPathGoal &goal);

    /**
     * @brief plan is the interface for implementation classes for 'non pose mode'
     * @param goal the requested goal message
     */
    virtual path_msgs::PathSequence planWithoutTargetPose(const path_msgs::PlanPathGoal &goal,
                                                 const lib_path::Pose2d& from_world, const lib_path::Pose2d& from_map);

    /**
     * @brief plan is the interface for implementation classes for 'pose mode'
     * @param goal the requested goal message
     * @param from_world start pose in world coordinates
     * @param to_world goal pose in world coordinates
     * @param from_map start pose in map coordinates
     * @param to_map goal pose in map coordinates
     */
    virtual path_msgs::PathSequence plan (const path_msgs::PlanPathGoal &goal,
                       const lib_path::Pose2d& from_world, const lib_path::Pose2d& to_world,
                       const lib_path::Pose2d& from_map, const lib_path::Pose2d& to_map) = 0;


    /**
     * @brief interpolatePath interpolates segments of the given path, if their length exeeds the given maximum distance
     * @param path path to interpolate
     * @param max_distance maximum segment length
     * @return interpolated path
     */
    path_msgs::PathSequence interpolatePath(const path_msgs::PathSequence &path, double max_distance);

    /**
     * @brief simplifyPath uses heuristics to try to simplify the given path
     * @param path The path to simplify.
     * @note Using this function only makes sense for non-kinematic paths!
     * @return  the simplified path.
     */
    path_msgs::PathSequence simplifyPath(const path_msgs::PathSequence &path);

    /**
     * @brief smoothPath smooths the given path
     * @param path path to smooth
     * @param weight_data weight for data integrity
     * @param weight_smooth weight for smoothness
     * @param tolerance iteration stopping criterium
     * @return smooted path
     */
    path_msgs::PathSequence smoothPath(const path_msgs::PathSequence& path, double weight_data, double weight_smooth, double tolerance = 0.000001);

    /**
     * @brief optimizePathCost generates a new path based on <path> by performing gradient descent with the current costmap.
     * @param path the path to optimize
     * @return optimized path
     */
    path_msgs::PathSequence optimizePathCost(const path_msgs::PathSequence& path);

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
    void publish(const path_msgs::PathSequence& path, const path_msgs::PathSequence &path_raw);

protected:
    void transformPose(const geometry_msgs::PoseStamped& pose, lib_path::Pose2d& world, lib_path::Pose2d& map);

    void preprocess(const path_msgs::PlanPathGoal& request);
    path_msgs::PathSequence postprocess(const path_msgs::PathSequence& path);

    void preempt();
    void feedback(int status);

    path_msgs::PathSequence empty() const;

protected:
    virtual bool supportsGoalType(int type) const = 0;

    void visualizeOutline(const geometry_msgs::Pose &at, int id, const std::string &frame);
    void visualizePath(const path_msgs::PathSequence& path, int id = 0, double alpha = 0.5);
    void visualizePathLine(const path_msgs::PathSequence &path, int id);

    geometry_msgs::PoseStamped lookupPose();
    tf::StampedTransform lookupTransform(const std::string& from, const std::string& to, const ros::Time& stamp);


private:
    void laserCallback(const sensor_msgs::LaserScanConstPtr& scan, bool front);
    void integrateLaserScan(const sensor_msgs::LaserScan &scan);

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud);
    void integratePointCloud(const sensor_msgs::PointCloud2 &cloud);

    void growObstacles(const path_msgs::PlanPathGoal &request, double radius);

    void calculateGradient(cv::Mat& gx, cv::Mat& gy);
    void publishGradient(const cv::Mat &gx, const cv::Mat &gy);

    path_msgs::PathSequence findPath(const path_msgs::PlanPathGoal &request);

    void planThreaded(const path_msgs::PlanPathGoal &request);
    path_msgs::PathSequence doPlan(const path_msgs::PlanPathGoal& request);


    path_msgs::DirectionalPath smoothPathSegment(const path_msgs::DirectionalPath &path, double weight_data, double weight_smooth, double tolerance);

    void subdividePath(path_msgs::DirectionalPath& result, geometry_msgs::PoseStamped low, geometry_msgs::PoseStamped up, double max_distance);

protected:
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv;

    bool is_cost_map_;

    bool use_map_topic_;
    bool use_cost_map_;
    bool use_cost_map_service_;
    bool use_map_service_;

    bool use_cloud_;
    bool use_scan_front_;
    bool use_scan_back_;


    bool pre_process_;
    bool post_process_;
    bool use_collision_gridmap_;

    bool post_process_optimize_cost_;
    bool publish_gradient_;
    double cost_optimization_weight_data;
    double cost_optimization_weight_smooth;
    double cost_optimization_weight_cost;
    double cost_optimization_tolerance;


    double grow_obstacles_;

    double size_forward;
    double size_backward;
    double size_width;

    ros::Subscriber goal_pose_sub;
    ros::Subscriber map_sub;
    ros::Subscriber sub_cloud;
    ros::Subscriber sub_front;
    ros::Subscriber sub_back;

    ros::ServiceClient map_service_client;
    ros::ServiceClient cost_map_service_client;

    actionlib::SimpleActionServer<path_msgs::PlanPathAction> server_;

    ros::Publisher viz_pub;
    ros::Publisher viz_array_pub;
    ros::Publisher cost_pub;
    ros::Publisher map_pub;
    tf::TransformListener tfl;

    std::string world_frame_;
    std::string robot_frame_;

    lib_path::SimpleGridMap2d * map_info;
    double map_rotation_yaw_;

    nav_msgs::OccupancyGridConstPtr pending_map;

    nav_msgs::OccupancyGrid cost_map;
    std::vector<double> gradient_x;
    std::vector<double> gradient_y;

    sensor_msgs::PointCloud2 cloud_;
    sensor_msgs::LaserScan scan_front;
    sensor_msgs::LaserScan scan_back;

    // threaded
    path_msgs::PathSequence thread_result;
    bool thread_running;
    boost::thread* thread_;
    boost::mutex thread_mutex;

    boost::mutex map_mutex;

protected:
    ros::Publisher path_publisher_;
    ros::Publisher raw_path_publisher_;
};

#endif // PLANNER_NODE_H
