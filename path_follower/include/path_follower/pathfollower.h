#ifndef PATHFOLLOWER_H
#define PATHFOLLOWER_H

/// THIRD PARTY
#include <Eigen/Core>
/// ROS
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <path_msgs/FollowPathAction.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

/// PROJECT
#include <path_follower/pathfollowerparameters.h>
#include <path_follower/utils/parameters.h>
#include <path_follower/utils/obstaclecloud.hpp> // TODO: this includes pcl...

/// SYSTEM
#include <memory>

class ControllerFactory;
class CoursePredictor;
class Visualizer;
class SupervisorChain;
class LocalPlanner;
class RobotController;
class Path;
class ObstacleAvoider;

//class ObstacleCloud;
class MoveCommand;

class PathFollower
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

public:
    PathFollower(ros::NodeHandle &nh);
    ~PathFollower();


    geometry_msgs::Twist getVelocity() const;
    bool transformToLocal(const geometry_msgs::PoseStamped& global, geometry_msgs::PoseStamped& local );
    bool transformToLocal(const geometry_msgs::PoseStamped& global, Eigen::Vector3d& local );
    bool transformToGlobal(const geometry_msgs::PoseStamped& local, geometry_msgs::PoseStamped& global );

    boost::variant<path_msgs::FollowPathFeedback, path_msgs::FollowPathResult> update();

    bool isRunning() const;
    void start();
    void stop(int status);
    void emergencyStop();

    void setGoal(const path_msgs::FollowPathGoal& goal);

    RobotController* getController();
    const PathFollowerParameters &getOptions() const;
    Visualizer& getVisualizer() const;

    ros::NodeHandle& getNodeHandle();
    boost::shared_ptr<ObstacleCloud const> getObstacleCloud() const;
    CoursePredictor &getCoursePredictor();
    Eigen::Vector3d getRobotPose() const;
    const geometry_msgs::Pose &getRobotPoseMsg() const;
    std::shared_ptr<Path> getPath();
    std::string getFixedFrameId();

    ROS_DEPRECATED void setStatus(int status);

    bool callObstacleAvoider(MoveCommand *cmd);

private:
    bool getWorldPose(Eigen::Vector3d *pose_vec, geometry_msgs::Pose* pose_msg = nullptr) const;


    //! Callback for odometry messages
    void odometryCB(const nav_msgs::OdometryConstPtr &odom);

    void obstacleCloudCB(const boost::shared_ptr<ObstacleCloud const>&);

    //! Update the current pose of the robot.
    /** @see robot_pose_, robot_pose_msg_ */
    bool updateRobotPose();


    /**
     * @brief Execute one path following iteration
     * @param feedback Feedback of the running path execution. Only meaningful, if return value is 1.
     * @param result Result of the finished path execution. Only meaningful, if return value is 0.
     * @return Returns `false` if the path execution is finished (no matter if successful or not) and `true` if it is still running.
     */
    bool execute(path_msgs::FollowPathFeedback& feedback, path_msgs::FollowPathResult& result);

    void setPath(const path_msgs::PathSequence& path);

    //! Split path into subpaths at turning points.
    void findSegments(const path_msgs::PathSequence& path, bool only_one_segment);

    //! Publish to the global path_points
    void publishPathMarker();

private:
    ros::NodeHandle node_handle_;

    //! Publisher for driving commands.
    ros::Publisher cmd_pub_;
    //! Publisher for local paths
    ros::Publisher local_path_pub_;
    //! Publisher for all local paths
    ros::Publisher whole_local_path_pub_;
    //! Publisher for the path points of the global path
    ros::Publisher g_points_pub_;

    //! Subscriber for odometry messages.
    ros::Subscriber odom_sub_;
    //! Subscriber for the obstacle point cloud (used by ObstacleAvoider).
    ros::Subscriber obstacle_cloud_sub_;

    tf::TransformListener pose_listener_;

    std::unique_ptr<ControllerFactory> controller_factory_;

    //! The robot controller is responsible for everything that is dependend on robot model and controller type.
    std::shared_ptr<RobotController> controller_;

    std::shared_ptr<LocalPlanner> local_planner_;

    std::shared_ptr<ObstacleAvoider> obstacle_avoider_;

    std::unique_ptr<SupervisorChain> supervisors_;

    //! Predict direction of movement for controlling and obstacle avoidance
    std::unique_ptr<CoursePredictor> course_predictor_;

    Visualizer* visualizer_;

    PathFollowerParameters opt_;

    //! The last received odometry message.
    nav_msgs::Odometry odometry_;

    //! The last received obstacle cloud
    boost::shared_ptr<ObstacleCloud const> obstacle_cloud_;

    //! Current pose of the robot as Eigen vector (x,y,theta).
    Eigen::Vector3d robot_pose_world_;
    Eigen::Vector3d robot_pose_odom_;
    //! Current pose of the robot as geometry_msgs pose.
    geometry_msgs::Pose robot_pose_world_msg_;
    geometry_msgs::Pose robot_pose_odom_msg_;
    //! Path driven by the robot
    visualization_msgs::Marker g_robot_path_marker_;

    //! Path as a list of separated subpaths
    std::shared_ptr<Path> path_;

    //! If set to a value >= 0, path execution is stopped in the next iteration. The value of pending_error_ is used as status code.
    int pending_error_;

    bool is_running_;

    //! Velocity for the Local Planner
    double vel_;
};

#endif // PATHFOLLOWER_H
