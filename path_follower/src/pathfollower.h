#ifndef PATHFOLLOWER_H
#define PATHFOLLOWER_H

/// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

/// PROJECT
#include <path_msgs/FollowPathAction.h>
#include <utils_robot/LaserEnvironment.h>
#include "BehaviouralPathDriver.h"
#include "obstacledetector.h"
#include "vector_field_histogram.h"


class PathFollower
{
public:
    PathFollower(ros::NodeHandle &nh);
    ~PathFollower();

    bool getWorldPose(Vector3d& pose, geometry_msgs::Pose* pose_out = NULL) const;
    geometry_msgs::Twist getVelocity() const;
    bool transformToLocal(const geometry_msgs::PoseStamped& global, geometry_msgs::PoseStamped& local );
    bool transformToLocal(const geometry_msgs::PoseStamped& global, Vector3d& local );
    bool transformToGlobal(const geometry_msgs::PoseStamped& local, geometry_msgs::PoseStamped& global );

    void update();

    /**
     * @brief Check if there is an obstacle in front of the robot.
     *
     * By default, only the laser scan is used, by calling LaserEnvironment::CheckCollision().
     * If parameter ~use_obstacle_map is set to true, the ObstacleDetector class is used instead.
     * See those classes for more details
     *
     * @param course_angle Angle of the current course (e.g. use steering angle).
     * @param box_length Length of the collision box. If an object is within this distance, an collision is thrown.
     * @param box_width Width of the collision box.
     * @param curve_enlarge_factor The width of the box is enlarged a bit in curves. This argument controls how much (it is misleadingly called 'length' in LaserEnvironment).
     * @return True, if there is an object within the collision box.
     * @see LaserEnvironment::CheckCollision() / ObstacleDetector for more details.
     */
    bool checkCollision(double course_angle, double box_length, double box_width = 0.3, double curve_enlarge_factor = 0.5);

    /**
     * @brief Check if there is an obstacle within a rectangular box in front of the robot.
     *
     * The box is placed in front of the laser and is defined by its width and length as displayed in the "figure" below:
     *
     *          +------------------+
     *  ##   ## |                  | |
     *  ####### |                  | width
     *  ####### |                  | |
     *  ##   ## |                  |
     *          +------------------+
     *   ^robot     <- length ->
     *
     *
     *
     * @param box_width Width of the box, which is checked for obstacles.
     * @param box_length Length of the box, which is checked for obstacles.
     * @return true, if there is an obstacle in the box.
     */
    bool simpleCheckCollision(float box_width, float box_length);

    VectorFieldHistogram& getVFH();

private:
    typedef actionlib::SimpleActionServer<path_msgs::FollowPathAction> FollowPathServer;

    ros::NodeHandle node_handle_;

    //! Action server that communicates with path_control (or who ever sends actions)
    FollowPathServer follow_path_server_;

    //! Publisher for driving commands.
    ros::Publisher cmd_pub_;
    //! Subscriber for odometry messages.
    ros::Subscriber odom_sub_;
    //! Subscriber for the obstacle grid map (used by ObstacleDetector).
    ros::Subscriber obstacle_map_sub_;
    //! Subscriber for laser scans (used for obstacle detection if no obstacle map is used).
    ros::Subscriber laser_sub_;

    MotionController *active_ctrl_;

    //! The last received odometry message.
    nav_msgs::Odometry odometry_;

    //! Name of the world frame (default: /map)
    std::string world_frame_;
    //! Name of the robot frame (default: /base_link)
    std::string robot_frame_;

    tf::TransformListener pose_listener_;

    //! If set to true, the obstacle map is used for obstacle detection, otherwise the laser scans.
    bool use_obstacle_map_;

    //! If set to true, vector field histogram is used for collision avoidance.
    bool use_vfh_;

    //! Obstacle detector working with obstacle map. Only used, if ~use_obstacle_map:=true
    ObstacleDetector obstacle_detector_;

    //! Provides obstacle detection based on the laser scans. Is used instead of ObstacleDetector, if ~use_obstacle_map:=false
    LaserEnvironment laser_env_;

    //! Last received laser scan (set by callback).
    sensor_msgs::LaserScan laser_scan_;

    //! Used for collision avoidance. Only used if ~use_obstacle_map:=true and ~use_vfh:=true.
    VectorFieldHistogram vfh_;


    void followPathGoalCB();
    void followPathPreemptCB();

    void odometryCB(const nav_msgs::OdometryConstPtr &odom);

    //! Callback for laser scan messages.
    void laserCB(const sensor_msgs::LaserScanConstPtr& scan);

    //! Callback for the obstacle grid map. Used by ObstacleDetector and VectorFieldHistorgram.
    void obstacleMapCB(const nav_msgs::OccupancyGridConstPtr& map);
};

#endif // PATHFOLLOWER_H
