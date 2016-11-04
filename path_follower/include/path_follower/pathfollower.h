#ifndef PATHFOLLOWER_H
#define PATHFOLLOWER_H

/// THIRD PARTY
#include <Eigen/Dense>

/// ROS
#include <path_msgs/FollowPathAction.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

/// PROJECT
#include <path_follower/pathfollowerparameters.h>

/// SYSTEM
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <memory>
#include <boost/variant.hpp>

class ControllerFactory;
class CoursePredictor;
class Visualizer;
class SupervisorChain;
class LocalPlanner;
class RobotController;
class Path;
class ObstacleAvoider;

class ObstacleCloud;
class MoveCommand;

class PoseTracker;

class PathFollower
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

public:
    PathFollower(ros::NodeHandle &nh);
    ~PathFollower();



    boost::variant<path_msgs::FollowPathFeedback, path_msgs::FollowPathResult> update();

    bool isRunning() const;
    void start();
    void stop(int status);
    void emergencyStop();

    void setGoal(const path_msgs::FollowPathGoal& goal);

    PoseTracker& getPoseTracker();

    RobotController* getController();
    const PathFollowerParameters &getOptions() const;
    Visualizer& getVisualizer() const;

    ros::NodeHandle& getNodeHandle();

    std::shared_ptr<ObstacleCloud const> getObstacleCloud() const;
    void obstacleCloudCB(const std::shared_ptr<ObstacleCloud const>&);

    CoursePredictor &getCoursePredictor();
    std::shared_ptr<Path> getPath();
    std::string getFixedFrameId();

    /*ROS_DEPRECATED*/ void setStatus(int status);

    bool callObstacleAvoider(MoveCommand *cmd);

private:
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

    //! Subscriber for the obstacle point cloud (used by ObstacleAvoider).


    std::shared_ptr<PoseTracker> pose_tracker_;

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

    //! The last received obstacle cloud
    std::shared_ptr<ObstacleCloud const> obstacle_cloud_;
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
