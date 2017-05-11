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
#include <path_follower/utils/path_follower_config.h>

/// SYSTEM
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <memory>
#include <boost/variant.hpp>

/// FORWARD DECLARATIONS
class FollowerFactory;
class CoursePredictor;
class Visualizer;
class SupervisorChain;
class AbstractLocalPlanner;
class RobotController;
class Path;
class CollisionAvoider;

class ObstacleCloud;
class MoveCommand;

class PoseTracker;
class PathFollowerParameters;
class LocalPlannerParameters;

/**
 * @brief The PathFollower class is a facade for the complete following subsystem.
 *        It keeps track of the current path and provides access to other components.
 */
class PathFollower
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

public:
    /**
     * @brief PathFollower
     * @param nh a global node handle to use
     */
    PathFollower(ros::NodeHandle &nh);

    /**
     * Destructior
     */
    ~PathFollower();

    /**
     * @brief update has to be called repeatedly while the current path is active.
     *        PathFollower itself does not loop internally.
     * @return FollowPathFeedback, if the goal is not reached, otherwise FollowPathResult
     */
    boost::variant<path_msgs::FollowPathFeedback, path_msgs::FollowPathResult> update();

    /**
     * @brief setObstacles updates the current obstacle cloud for the follower
     * @param cloud is the latest obstacle cloud
     */
    void setObstacles(const std::shared_ptr<ObstacleCloud const>& cloud);

    /**
     * @brief isRunning returns the current state
     * @return true, iff a path is being followed
     */
    bool isRunning() const;

    /**
     * @brief stop halts the robot.
     * @param status is the path following status, used to communicate why the robot is stopped
     */
    void stop(int status);

    /**
     * @brief emergencyStop is equivalent to stop(FollowPathResult::RESULT_STATUS_INTERNAL_ERROR)
     */
    void emergencyStop();

    /**
     * @brief setGoal initializes a new path to follow
     * @param goal configuration for the current path
     */
    void setGoal(const path_msgs::FollowPathGoal& goal);

    /**
     * @brief getPoseTracker accesses the pose tracker
     * @return the pose tracker
     */
    PoseTracker& getPoseTracker();

    /**
     * @brief getControllerFactory accesses the controller factory
     * @return  the controller factory
     */
    FollowerFactory& getFollowerFactory();

    /**
     * @brief getOptions accesses the path following options
     * @return  the path following options
     */
    const PathFollowerParameters &getOptions() const;

    /**
     * @brief getNodeHandle accesses the global node handle
     * @return the global node handle
     */
    ros::NodeHandle& getNodeHandle();

private:
    /**
     * @brief Execute one path following iteration
     * @param feedback Feedback of the running path execution. Only meaningful, if return value is 1.
     * @param result Result of the finished path execution. Only meaningful, if return value is 0.
     * @return Returns `false` if the path execution is finished (no matter if successful or not) and `true` if it is still running.
     */
    bool execute(path_msgs::FollowPathFeedback& feedback, path_msgs::FollowPathResult& result);

    //! Sets the current path
    void setPath(const path_msgs::PathSequence& path);

    //! Split path into subpaths at turning points.
    void findSegments(const path_msgs::PathSequence& path, bool only_one_segment);

    //! Publish to the global path_points
    void publishPathMarker();

    //! Converts a goal to a configuration name
    PathFollowerConfigName goalToConfig(const path_msgs::FollowPathGoal &goal) const;

    //! Start following the current path
    void start();

    //! Gets the name of the currently used fixed frame.
    std::string getFixedFrameId() const;

private:
    //! Global ros node handle
    ros::NodeHandle node_handle_;

    //! Publisher for driving commands.
    ros::Publisher cmd_pub_;
    //! Publisher for local paths
    ros::Publisher local_path_pub_;
    //! Publisher for all local paths
    ros::Publisher whole_local_path_pub_;
    //! Publisher for the path points of the global path
    ros::Publisher marker_pub_;

    //! The pse tracker keeps track of tf information
    std::shared_ptr<PoseTracker> pose_tracker_;

    //! The controller factory is used to create controllers, local planners and obstacle avoiders
    std::unique_ptr<FollowerFactory> follower_factory_;

    //! The currently used config for path following, set by the latest goal
    std::shared_ptr<PathFollowerConfig> current_config_;

    //! Cache of already used configs
    std::map<PathFollowerConfigName, std::shared_ptr<PathFollowerConfig>> config_cache_;

    //! The currently active supervisors
    std::unique_ptr<SupervisorChain> supervisors_;

    //! Predict direction of movement for controlling and obstacle avoidance
    std::unique_ptr<CoursePredictor> course_predictor_;

    //! Helper class for visualizing things in RViz
    Visualizer* visualizer_;

    //! All general path following parameters
    const PathFollowerParameters& opt_;

    const LocalPlannerParameters& opt_l_;

    //! The last received obstacle cloud
    std::shared_ptr<ObstacleCloud const> obstacle_cloud_;
    //! Path driven by the robot
    visualization_msgs::Marker g_robot_path_marker_;

    //! Path as a list of separated subpaths
    std::shared_ptr<Path> path_;

    //! If set to a value >= 0, path execution is stopped in the next iteration. The value of pending_error_ is used as status code.
    int pending_error_;

    //! Flag for global en-/disabling of the follower
    bool is_running_;

    //! Velocity for the Local Planner
    double vel_;
};

#endif // PATHFOLLOWER_H
