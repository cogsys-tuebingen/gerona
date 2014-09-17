#ifndef PATHFOLLOWER_H
#define PATHFOLLOWER_H

/// THIRD PARTY
#include <Eigen/Core>

/// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>

/// PROJECT
#include <path_msgs/FollowPathAction.h>
#include <utils_robot/LaserEnvironment.h>
#include <utils_general/Global.h>
#include "obstacledetectorackermann.h"
#include "obstacledetectoromnidrive.h"
#include "vector_field_histogram.h"
#include "robotcontroller.h"
#include "pathlookout.h"
#include "PidCtrl.h"
#include "vector_field_histogram.h"
#include "visualizer.h"
#include "path.h"
#include "coursepredictor.h"


//Forward declaration
class Behaviour;


class PathFollower
{

public:
    friend class Behaviour;

    struct Options
    {
        Options()
        { reset(); }

        void reset() {
            path_idx = 0;
            wp_idx = 0;
        }

        double wp_tolerance_;
        double goal_tolerance_;

        //! Minimum speed of the robot (needed, as the outdoor buggys can't handle velocities below about 0.3).
        float min_velocity_;
        //! Maximum velocity (to prevent the high level control from running amok).
        float max_velocity_;

        //! Maximum distance the robot is allowed to depart from the path. If this threshold is exceeded,
        //! the path follower will abort.
        double max_distance_to_path_;

        double steer_slow_threshold_;

        //! Width of the collisin box for obstacle avoidance.
        float collision_box_width_;
        //! Minimum length of the collision box for obstacle avoidance (grows with increasing velocity).
        float collision_box_min_length_;
        float collision_box_crit_length_;
        //! Maximum length of the collision box for obstacle avoidance.
        float collision_box_max_length_;
        //! This factor determines, how much the length of the box is increased, depending on the velocity.
        float collision_box_velocity_factor_;
        //! The velocity for which the maximum length should be used
        float collision_box_velocity_saturation_;

        //! Name of the world frame (default: /map)
        std::string world_frame_;
        //! Name of the robot frame (default: /base_link)
        std::string robot_frame_;

        //! If set to true, the obstacle map is used for obstacle detection, otherwise the laser scans.
        bool use_obstacle_map_;

        //! If set to true, vector field histogram is used for collision avoidance.
        bool use_vfh_;

        //TODO: those are not really options. maybe move them somewhere else?
        int path_idx;
        int wp_idx;
    };


public:
    PathFollower(ros::NodeHandle &nh);
    ~PathFollower();

    bool getWorldPose(Vector3d *pose_vec, geometry_msgs::Pose* pose_msg = NULL) const;
    geometry_msgs::Twist getVelocity() const;
    bool transformToLocal(const geometry_msgs::PoseStamped& global, geometry_msgs::PoseStamped& local );
    bool transformToLocal(const geometry_msgs::PoseStamped& global, Vector3d& local );
    bool transformToGlobal(const geometry_msgs::PoseStamped& local, geometry_msgs::PoseStamped& global );

    void spin();

    void update();

    bool checkCollision(double course);

    VectorFieldHistogram& getVFH();

    RobotController* getController();

    PathLookout* getPathLookout();

    CoursePredictor &getCoursePredictor();

    //! Send 'text' to a text to speech processor.
    void say(std::string text);

    Eigen::Vector3d getRobotPose() const;
    const geometry_msgs::Pose &getRobotPoseMsg() const;

    Behaviour* getActiveBehaviour() const
    {
        return active_behaviour_;
    }

    const PathFollower::Options &getOptions() const
    {
        return opt_;
    }

private:
    typedef actionlib::SimpleActionServer<path_msgs::FollowPathAction> FollowPathServer;

    ros::NodeHandle node_handle_;

    //! Action server that communicates with path_control (or who ever sends actions)
    FollowPathServer follow_path_server_;

    //! Publisher for driving commands.
    ros::Publisher cmd_pub_;
    //! Publisher for text to speech messages.
    ros::Publisher speech_pub_;
    //! Publisher for beeps.
    ros::Publisher beep_pub_;

    //! Subscriber for odometry messages.
    ros::Subscriber odom_sub_;
    //! Subscriber for the obstacle grid map (used by ObstacleDetector).
    ros::Subscriber obstacle_map_sub_;
    //! Subscriber for laser scans (used for obstacle detection if no obstacle map is used).
    ros::Subscriber laser_sub_;

    tf::TransformListener pose_listener_;

    //! The robot controller is responsible for everything that is dependend on robot model and controller type.
    RobotController *controller_;

    //! Active behaviour used to follow the path.
    Behaviour* active_behaviour_;

    //! Look for obstacles on the path ahead of the robot.
    PathLookout path_lookout_;

    //! Predict direction of movement for controlling and obstacle avoidance
    CoursePredictor course_predictor_;

    //! Provides obstacle detection based on the laser scans. Is used instead of ObstacleDetector, if ~use_obstacle_map:=false
    LaserEnvironment laser_env_;

    //! Used for collision avoidance. Only used if ~use_obstacle_map:=true and ~use_vfh:=true.
    VectorFieldHistogram vfh_;

    Visualizer* visualizer_;

    Options opt_;

    //! The last received odometry message.
    nav_msgs::Odometry odometry_;
    //! Last received laser scan (set by callback).
    sensor_msgs::LaserScan laser_scan_;

    //! Current pose of the robot as Eigen vector (x,y,theta).
    Eigen::Vector3d robot_pose_;
    //! Current pose of the robot as geometry_msgs pose.
    geometry_msgs::Pose robot_pose_msg_;

    //! Full path
    nav_msgs::Path path_;
    //! Path as a list of separated subpaths
    std::vector<Path> paths_;

    //! If set to a value >= 0, path execution is stopped in the next iteration. The value of pending_error_ is used as status code.
    int pending_error_;

    ros::Time last_beep_;
    ros::Duration beep_pause_;


    //! Callback for new follow_path action goals.
    void followPathGoalCB();
    //! Callback for follow_path action preemption.
    void followPathPreemptCB();

    //! Callback for odometry messages
    void odometryCB(const nav_msgs::OdometryConstPtr &odom);

    //! Callback for laser scan messages.
    void laserCB(const sensor_msgs::LaserScanConstPtr& scan);

    //! Callback for the obstacle grid map. Used by ObstacleDetector and VectorFieldHistorgram.
    void obstacleMapCB(const nav_msgs::OccupancyGridConstPtr& map);


    //! Remove the active behaviour.
    void clearActive();

    //! Publish beep commands.
    void beep(const std::vector<int>& beeps);

    //! Update the current pose of the robot.
    /** @see robot_pose_, robot_pose_msg_ */
    bool updateRobotPose();

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
    bool isObstacleInBox(double course_angle, double box_length, double box_width = 0.3, double curve_enlarge_factor = 0.5);

    //! Start driving on the path
    void start();

    //! Stop driving on the path
    void stop();

    /**
     * @brief Execute one path following iteration, using the active behaviour.
     * @param feedback Feedback of the running path execution. Only meaningful, if return value is 1.
     * @param result Result of the finished path execution. Only meaningful, if return value is 0.
     * @return Returns `false` if the path execution is finished (no matter if successful or not) and `true` if it is still running.
     */
    bool executeBehaviour(path_msgs::FollowPathFeedback& feedback, path_msgs::FollowPathResult& result);

    void configure();

    void setGoal(const path_msgs::FollowPathGoal& goal);
    void setPath(const nav_msgs::Path& path);

    //! Split path into subpaths at turning points.
    void findSegments(bool only_one_segment);
};

#endif // PATHFOLLOWER_H
