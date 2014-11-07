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
#include <utils_general/Global.h>
#include <path_follower/obstacle_avoidance/obstacledetectorackermann.h>
#include <path_follower/obstacle_avoidance/obstacledetectoromnidrive.h>
#include <path_follower/legacy/vector_field_histogram.h>
#include <path_follower/controller/robotcontroller.h>
#include <path_follower/utils/PidCtrl.h>
#include <path_follower/legacy/vector_field_histogram.h>
#include <path_follower/utils/visualizer.h>
#include <path_follower/utils/path.h>
#include <path_follower/utils/coursepredictor.h>
#include <path_follower/utils/parameters.h>

#include <path_follower/supervisor/supervisorchain.h>
#include <path_follower/supervisor/pathlookout.h>

class PathFollower
{

public:
    friend class Behaviour;

    struct PathIndex //TODO: better name...
    {
        PathIndex()
        { reset(); }

        void reset() {
            path_idx = 0;
            wp_idx = 0;
        }

        int path_idx;
        int wp_idx;
    };


    struct Options : public Parameters
    {
        P<std::string> controller;
        P<double> wp_tolerance;
        P<double> goal_tolerance;
        P<float> min_velocity;
        P<float> max_velocity;
        P<double> steer_slow_threshold;
        P<double> max_distance_to_path;
        P<float> collision_box_width;
        P<float> collision_box_min_length;
        P<float> collision_box_crit_length;
        P<float> collision_box_max_length;
        P<float> collision_box_velocity_factor;
        P<float> collision_box_velocity_saturation;
        P<std::string> world_frame;
        P<std::string> robot_frame;
        P<bool> use_obstacle_map;
        P<bool> use_vfh;
        P<bool> use_path_lookout;
        P<bool> abort_if_obstacle_ahead;

        Options():
            controller(this, "~controller", "ackermann_pid", "Defines, which controller is used."),
            wp_tolerance(this,  "~waypoint_tolerance",  0.20 , ""),
            goal_tolerance(this,  "~goal_tolerance",  0.15 , ""),
            steer_slow_threshold(this,  "~steer_slow_threshold",  0.25 , ""),
            max_distance_to_path(this,  "~max_distance_to_path",  0.3 , "Maximum distance the robot is allowed to depart from the path. If this threshold is exceeded, the path follower will abort."),

            world_frame(this, "~world_frame",  "/map", "Name of the world frame."),
            robot_frame(this, "~robot_frame",  "/base_link", "Name of the robot frame."),

            use_obstacle_map(this, "~use_obstacle_map",  false, "If set to true, the obstacle map is used for obstacle detection, otherwise the laser scans."),
            use_vfh(this, "~use_vfh",  false, "If set to true, vector field histogram is used for collision avoidance."),
            use_path_lookout(this, "~use_path_lookout",  true, "If set to true, path lookout is done (check if there are obstacles somewhere on the path ahead of the robot)."),

            min_velocity(this,  "~min_velocity",  0.4 , "Minimum speed of the robot (needed, as the outdoor buggys can't handle velocities below about 0.3)."),
            max_velocity(this,  "~max_velocity",  2.0 , "Maximum velocity (to prevent the high level control from running amok)."),

            collision_box_width(this,  "~collision_box_width",  0.5, "Width of the collision box for obstacle avoidance."),
            collision_box_min_length(this,  "~collision_box_min_length",  0.5, "Minimum length of the collision box for obstacle avoidance (grows with increasing velocity)."),
            collision_box_crit_length(this,  "~collision_box_crit_length",  0.3, ""),
            collision_box_max_length(this,  "~collision_box_max_length",  1.0, "Maximum length of the collision box for obstacle avoidance."),
            collision_box_velocity_factor(this,  "~collision_box_velocity_factor",  1.0, "This factor determines, how much the length of the box is increased, depending on the velocity."),
            collision_box_velocity_saturation(this,  "~collision_box_velocity_saturation",  max_velocity(), "The velocity for which the maximum length should be used."),
            abort_if_obstacle_ahead(this, "~abort_if_obstacle_ahead",  false, "If set to true, path execution is aborted, if an obstacle is detected on front of the robot. If false, the robot will stop, but not abort (the obstacle might move away).")
        {
            if(max_velocity() < min_velocity()) {
                ROS_ERROR("min velocity larger than max velocity!");
                max_velocity.set(min_velocity());
            }
            if(collision_box_max_length() < collision_box_min_length()) {
                ROS_ERROR("min length larger than max length!");
                collision_box_min_length.set(collision_box_max_length());
            }
            if(collision_box_min_length() < collision_box_crit_length()) {
                ROS_ERROR("min length smaller than crit length!");
                collision_box_crit_length.set(collision_box_min_length());
            }
        }
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

    bool isObstacleAhead(double course);

    VectorFieldHistogram& getVFH();

    RobotController* getController();

    CoursePredictor &getCoursePredictor();

    //! Send 'text' to a text to speech processor.
    void say(std::string text);

    Eigen::Vector3d getRobotPose() const;
    const geometry_msgs::Pose &getRobotPoseMsg() const;

    PathWithPosition::Ptr getPathWithPosition();

    void setStatus(int status);

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
    ros::Subscriber laser_front_sub_;
    ros::Subscriber laser_back_sub_;

    tf::TransformListener pose_listener_;

    //! The robot controller is responsible for everything that is dependend on robot model and controller type.
    RobotController *controller_;

    SupervisorChain supervisors_;

    //! Predict direction of movement for controlling and obstacle avoidance
    CoursePredictor course_predictor_;

    //! Used for collision avoidance. Only used if ~use_obstacle_map:=true and ~use_vfh:=true.
    VectorFieldHistogram vfh_;

    Visualizer* visualizer_;

    Options opt_;
    PathIndex path_idx_;

    //! The last received odometry message.
    nav_msgs::Odometry odometry_;

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

    bool is_running_;


    //! Callback for new follow_path action goals.
    void followPathGoalCB();
    //! Callback for follow_path action preemption.
    void followPathPreemptCB();

    //! Callback for odometry messages
    void odometryCB(const nav_msgs::OdometryConstPtr &odom);

    //! Callback for laser scan messages.
    void laserCB(const sensor_msgs::LaserScanConstPtr& scan, bool isBack=false);

    //! Callback for the obstacle grid map. Used by ObstacleDetector and VectorFieldHistorgram.
    void obstacleMapCB(const nav_msgs::OccupancyGridConstPtr& map);

    //! Publish beep commands.
    void beep(const std::vector<int>& beeps);

    //! Update the current pose of the robot.
    /** @see robot_pose_, robot_pose_msg_ */
    bool updateRobotPose();

    //! Start driving on the path
    void start();

    //! Stop driving on the path
    void stop();

    /**
     * @brief Execute one path following iteration
     * @param feedback Feedback of the running path execution. Only meaningful, if return value is 1.
     * @param result Result of the finished path execution. Only meaningful, if return value is 0.
     * @return Returns `false` if the path execution is finished (no matter if successful or not) and `true` if it is still running.
     */
    bool execute(path_msgs::FollowPathFeedback& feedback, path_msgs::FollowPathResult& result);

    void setGoal(const path_msgs::FollowPathGoal& goal);
    void setPath(const nav_msgs::Path& path);

    //! Split path into subpaths at turning points.
    void findSegments(bool only_one_segment);
};

#endif // PATHFOLLOWER_H
