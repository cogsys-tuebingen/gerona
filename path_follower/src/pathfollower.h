#ifndef PATHFOLLOWER_H
#define PATHFOLLOWER_H

/// THIRD PARTY
#include <Eigen/Core>

/// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>

/// PROJECT
#include <path_msgs/FollowPathAction.h>
#include <utils_robot/LaserEnvironment.h>
#include "BehaviouralPathDriver.h"
#include "obstacledetectorackermann.h"
#include "obstacledetectoromnidrive.h"
#include "vector_field_histogram.h"
#include "robotcontroller.h"
#include "pathlookout.h"


class PathFollower
{
public:
    PathFollower(ros::NodeHandle &nh);
    ~PathFollower();

    bool getWorldPose(Vector3d *pose_vec, geometry_msgs::Pose* pose_msg = NULL) const;
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

    VectorFieldHistogram& getVFH();

    RobotController* getController();

    PathLookout* getPathLookout();

    //! Send 'text' to a text to speech processor.
    void say(std::string text);

    Eigen::Vector3d getRobotPose() const;
    const geometry_msgs::Pose &getRobotPoseMsg() const;

private:
    typedef actionlib::SimpleActionServer<path_msgs::FollowPathAction> FollowPathServer;

    ros::NodeHandle node_handle_;

    //! Action server that communicates with path_control (or who ever sends actions)
    FollowPathServer follow_path_server_;

    //! Publisher for driving commands.
    ros::Publisher cmd_pub_;
    //! Publisher for text to speech messages.
    ros::Publisher speech_pub_;
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

    //! Provides obstacle detection based on the laser scans. Is used instead of ObstacleDetector, if ~use_obstacle_map:=false
    LaserEnvironment laser_env_;

    //! Last received laser scan (set by callback).
    sensor_msgs::LaserScan laser_scan_;

    //! Used for collision avoidance. Only used if ~use_obstacle_map:=true and ~use_vfh:=true.
    VectorFieldHistogram vfh_;

    RobotController *controller_;

    PathLookout path_lookout_;

    //! Current pose of the robot as Eigen vector (x,y,theta).
    Eigen::Vector3d robot_pose_;

    //! Current pose of the robot as geometry_msgs pose.
    geometry_msgs::Pose robot_pose_msg_;


    void followPathGoalCB();
    void followPathPreemptCB();

    void odometryCB(const nav_msgs::OdometryConstPtr &odom);

    //! Callback for laser scan messages.
    void laserCB(const sensor_msgs::LaserScanConstPtr& scan);

    //! Callback for the obstacle grid map. Used by ObstacleDetector and VectorFieldHistorgram.
    void obstacleMapCB(const nav_msgs::OccupancyGridConstPtr& map);

    bool updateRobotPose();
};

#endif // PATHFOLLOWER_H
