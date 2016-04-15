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
#include <path_follower/local_planner/local_planner.h>
#include <path_msgs/FollowPathAction.h>
#include <utils_general/Global.h>
#include <path_follower/pathfollowerparameters.h>
#include <path_follower/obstacle_avoidance/obstacledetectorackermann.h>
#include <path_follower/obstacle_avoidance/obstacledetectoromnidrive.h>
#include <path_follower/controller/robotcontroller.h>
#include <path_follower/utils/visualizer.h>
#include <path_follower/utils/path.h>
#include <path_follower/utils/coursepredictor.h>
#include <path_follower/utils/parameters.h>
#include <path_follower/obstacle_avoidance/obstacleavoider.h>
#include <path_follower/supervisor/supervisorchain.h>

/// SYSTEM
#include <memory>

class PathFollower
{
    friend class Behaviour;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

public:
    PathFollower(ros::NodeHandle &nh);
    ~PathFollower();


    geometry_msgs::Twist getVelocity() const;
    bool transformToLocal(const geometry_msgs::PoseStamped& global, geometry_msgs::PoseStamped& local );
    bool transformToLocal(const geometry_msgs::PoseStamped& global, Vector3d& local );
    bool transformToGlobal(const geometry_msgs::PoseStamped& local, geometry_msgs::PoseStamped& global );

    void spin();

    void update();

    RobotController* getController();

    CoursePredictor &getCoursePredictor();

    //! Send 'text' to a text to speech processor.
    void say(std::string text);

    Eigen::Vector3d getRobotPose() const;
    const geometry_msgs::Pose &getRobotPoseMsg() const;

    Path::Ptr getPath();
    std::string getFixedFrameId();

    ROS_DEPRECATED void setStatus(int status);

    bool callObstacleAvoider(MoveCommand *cmd);

    const PathFollowerParameters &getOptions() const;

    ros::NodeHandle& getNodeHandle();

private:
    bool getWorldPose(Vector3d *pose_vec, geometry_msgs::Pose* pose_msg = nullptr) const;

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
    //! Publisher for local paths
    ros::Publisher local_path_pub_;

    //! Subscriber for odometry messages.
    ros::Subscriber odom_sub_;
    //! Subscriber for the obstacle point cloud (used by ObstacleAvoider).
    ros::Subscriber obstacle_cloud_sub_;

    tf::TransformListener pose_listener_;

    //! The robot controller is responsible for everything that is dependend on robot model and controller type.
    std::shared_ptr<RobotController> controller_;

    std::shared_ptr<LocalPlanner> local_planner_;

    std::shared_ptr<ObstacleAvoider> obstacle_avoider_;

    SupervisorChain supervisors_;

    //! Predict direction of movement for controlling and obstacle avoidance
    CoursePredictor course_predictor_;

    Visualizer* visualizer_;

    PathFollowerParameters opt_;

    //! The last received odometry message.
    nav_msgs::Odometry odometry_;

    //! The last received obstacle cloud
    ObstacleCloud::ConstPtr obstacle_cloud_;

    //! Current pose of the robot as Eigen vector (x,y,theta).
    Eigen::Vector3d robot_pose_world_;
    Eigen::Vector3d robot_pose_odom_;
    //! Current pose of the robot as geometry_msgs pose.
    geometry_msgs::Pose robot_pose_world_msg_;
    geometry_msgs::Pose robot_pose_odom_msg_;

    //! Path as a list of separated subpaths
    Path::Ptr path_;

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

    void obstacleCloudCB(const ObstacleCloud::ConstPtr&);

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
    void findSegments(const nav_msgs::Path& path, bool only_one_segment);
};

#endif // PATHFOLLOWER_H
