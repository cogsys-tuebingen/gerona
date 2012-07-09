#ifndef PATHFOLLOWER_H
#define PATHFOLLOWER_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <motion_control/MotionAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>
#include <nav_msgs/OccupancyGrid.h>
#include <dynamic_reconfigure/server.h>

#define EIGEN_USE_NEW_STDVECTOR
#include <Eigen/Core>
#include <Eigen/StdVector>
#include "point_types.h"

#include "traversable_path/follow_pathConfig.h"

// forward declarations
class MapProcessor;
class MarkerPublisher;
class TimeoutLocker;


/**
 * @brief Main class of the follow_path node.
 *
 * The class subscribes for the terrain classification of the node classify_path, determines goal points for navigation
 * and commands the robot to drive to this goals (using the motion_control package).
 *
 * @author Felix Widmaier
 * @version $Id$
 */
class PathFollower
{
public:
    typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > vectorVector2f;

    PathFollower();
    ~PathFollower();

private:
    //! Pose of the robot with position and direction.
    struct RobotPose {
        //! Position as point.
        Eigen::Vector2f position;
        //! Orientation as vector.
        Eigen::Vector2f direction;
    };

    struct Goal {
        //! This is true, if the current goal is still active. If it is false, the robot does not drive to this goal.
        bool is_set;
        //! Coordinates of the goal (frame_id = /map)
        Eigen::Vector2f position;
        //! Angle of the goal orientation.
        float theta;

        Goal(): is_set(false), position(0.0,0.0), theta(0.0) {}
        Goal(Eigen::Vector2f pos, float theta, bool is_set=true) {
            this->position = pos;
            this->theta = theta;
            this->is_set = is_set;
        }
    };

    /**
     * @brief Represents a line in parametric form.
     *
     * This struct is meant for use with PathFollower::fitLinear(). It contains a point of the line, a direction
     * vector and a normal vector. The vectors are expected to be normalized.
     * Furthermore there is a soundness value. The smaller this value is, the closer fits the line to the points.
     */
    struct Line {
        //! A point lying on the line.
        Eigen::Vector2f point;
        //! Direction vector.
        Eigen::Vector2f direction;
        //! Soundness of the fitted line (smaller values means better fitting).
        float soundness;

        Line() {
            direction = point = Eigen::Vector2f::Zero();
            soundness = 0;
        }
    };

    ros::NodeHandle node_handle_;
    //! Subscriber for the traversability map.
    ros::Subscriber subscribe_map_;
    //! Publisher for rvis markers.
    MarkerPublisher *rviz_marker_;
    //! Listener for tf data.
    tf::TransformListener tf_listener_;
    //! Sends commands to motion_control
    actionlib::SimpleActionClient<motion_control::MotionAction> motion_control_action_client_;
    //! dynamic reconfigure server.
    dynamic_reconfigure::Server<traversable_path::follow_pathConfig> reconfig_server_;

    //! The current goal.
    Goal current_goal_;

    /**
     * @brief Tells if the last cal of the map callback encountered an obstacle.
     * Always remember last obstacle-state to recognize if the current obstacle is still the same than in the method
     * call before.
     */
    bool obstacle_at_last_callback_;

    //! The traversability map.
    nav_msgs::OccupancyGridConstPtr map_;

    //! Pose of the robot.
    RobotPose robot_pose_;
    //! Path middle line.
    Line path_middle_line_;

    //! Filtered path angle.
    float path_angle_;

    //! Does image processing on the map.
    MapProcessor *map_processor_;

    //! Dynamic reconfigure values.
    traversable_path::follow_pathConfig config_;

    /**
     * @brief Locks/unlocks the goal.
     * Locked means, that new goals will be ignored until the goal is reached or the timeout expired.
     */
    TimeoutLocker *goal_locker_;

    void dynamicReconfigureCallback(const traversable_path::follow_pathConfig &config, uint32_t level);

    /**
     * @brief Calculate path direction and set goal point.
     *
     * This is the callback function for the traversability map subscriber. It takes the map, calculates the direction
     * of the current path and sets a new goalpoint for motion_control.
     *
     * @param msg A traversability map.
     */
    void mapCallback(const nav_msgs::OccupancyGridConstPtr &msg);

    void motionControlDoneCallback(const actionlib::SimpleClientGoalState& state,
                                   const motion_control::MotionResultConstPtr& result);
    void motionControlFeedbackCallback(const motion_control::MotionFeedbackConstPtr& feedback);

    /**
     * @brief Set the given position as navigation goal.
     *
     * Publishes a goal message for motion_control and a goal marker for rviz.
     * The position and angle are expected to be respective to frame "/map".
     *
     * If the new goal is too near at the current goal, it will not be set. This is necessary due to the way how
     * motion_control works. An exception is made if lock_goal is set.
     *
     * @param pos Position of the goal.
     * @param theta Angle of the goal orientation.
     * @param lock_goal If this value is true, the new goal will be locked. That is other goals will be ignored until
     *                  this goal is reached or a certain time has elapsed. Furthermore this option forces the new goal
     *                  to be set no matter if the goal is too near to the current goal or not.
     * @param velocity Velocity of the robot when driving to this goal. If set to -1.0 the default value set by dynamic
     *                 reconfigure will be used.
     */
    void setGoal(Eigen::Vector2f pos, float theta, bool lock_goal=false, float velocity = -1.0);

    /**
     * @brief Refreshes all cached values (like robot pose and path line).
     * @see refreshRobotPose(), refreshPathLine(), refreshPathDirectionAngle()
     * @return True if all refreshes succeed, false if an error occures.
     */
    void refreshAll();

    /**
     * @brief Refresh the current position and orientation of the robot.
     *
     * Gets the current position and orientation of the robot and stores it to robot_pose_.
     * Use robot_pose_ to access the pose, after refreshing it with this method.
     *
     * @see robot_pose_
     * @return False if some failure occurs, otherwise true.
     */
    void refreshRobotPose();

    /**
     * @brief Refresh the path middle line.
     *
     * Calculates a line that fits to the direction of the path in front of the robot. The result is written to
     * PathFollower::path_middle_line_ so use this class member to access the line after refreshing it.
     *
     * Please note: This method depends on the current robot pose so call refreshRobotPose() first.
     * @see refreshRobotPose()
     * @see path_middle_line_
     */
    void refreshPathLine();

    /**
     * @brief Calculate the angle of the path direction (related to frame /map).
     *
     * Calculates the angle of the path depending on the current path middle line and stores the filtered value to
     * path_angle_.
     * As a side effect this method negates the middel line direction vector if the robot is looking in the other
     * direction.
     *
     * @see path_angle_
     */
    void refreshPathDirectionAngle();

    /**
     * @brief Find some points of the moddle of the curretn path.
     *
     * Searches for points of the middle of the path. This points can then be used to fit a "path middle line" to them.
     * @param out Output parameter. The points will be stored to this vector.
     * @return True on success, False if some failure occures (e.g. transform failes).
     */
    bool findPathMiddlePoints(vectorVector2f *out) const;

    /**
     * @brief Find the direction of the path, when standing in front of an obstacle.
     *
     * Assuming the robot has stopped in front of an obstacle (e.g. at a crossing or a dead end), this method tries
     * to find the best direction to go on.
     *
     * @return Path direction as normalized 2-dim. vector. If there is no possible direction, an zero-vector is
     *         returned.
     */
    Eigen::Vector2f findBestPathDirection() const;

    /**
     * @brief Get weight of the specified angle.
     *
     * This is a helper method for findBestPathDirection(). It is not meant to be used in an other context.
     * @param angle An angle (radian) of the intervall [-pi,pi].
     * @return The weight of this angle.
     */
    static float helperAngleWeight(float angle);

    /**
     * @brief Handles the navigation if the robot stands in front of an obstacle.
     *
     * If the robot stands in front of an obstacle (which is also the case at crossigns and dead ends), the direction of
     * the path (and thus a new goal) is determined. Before moving to the new goal, the robot first has to move a bit
     * backward to have enought space to maneuver.
     */
    void handleObstacle();

    //! Stop immediatly and cancle current goal.
    void stopRobot();

    /**
     * @brief Fit a line to a set of points (in 2-dim. space).
     *
     * The resulting line is returned in parametric form, but it also contains the normal vector.
     * Both, direction vector and normal vector are normalized to length 1.
     *
     * @param points A set of points, the line shall be fitted to.
     * @param result Output parameter with will contain the parameters of the resulting line.
     * @see PathFollower::Line
     */
    static void fitLinear(const PathFollower::vectorVector2f &points, Line *result);

    /**
     * @brief Calculate the vector that points from 'point' to 'line'.
     *
     * The resulting vector is a normal vector of the line with length equal to the distant of the point to the line
     * and pointing from the point to the line.
     *
     * @param line A line.
     * @param point A point.
     * @return Vector that points from point to line.
     */
    Eigen::Vector2f vectorFromPointToLine(const Line &line, const Eigen::Vector2f &point) const;

    /**
     * @brief Convert an Eigen::Vector2f to a geometry_msgs::Point.
     *
     * The z-value of the point is set to 0.
     *
     * @param v Some vector.
     * @return a Point with the values of the given vector.
     */
    static geometry_msgs::Point vectorToPoint(Eigen::Vector2f v);
};

#endif // PATHFOLLOWER_H
