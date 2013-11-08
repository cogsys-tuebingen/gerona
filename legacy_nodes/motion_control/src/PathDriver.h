/**
 * @file PathDriver.h
 * @date March 2012
 * @author marks
 */

#ifndef PATHDRIVER_H
#define PATHDRIVER_H

// C/C++
#include <vector>

// ROS/Eigen
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <visualization_msgs/Marker.h>

// Project
#include "DualPidCtrl.h"
#include "PidCtrl.h"
#include "MotionController.h"

namespace motion_control {

class PathDriver : public MotionController
{
public:
    /**
     * @brief Represents a position/speed pair.
     */
    struct Waypoint {
        /// Position of the waypoint
        geometry_msgs::PoseStamped pose;

        /// Veclocity we should have reached at this waypoint
        double speed;

        /**
         * @brief Initializes pose and velocity.
         * @param p The pose.
         * @param v The velocity.
         */
        Waypoint( const geometry_msgs::PoseStamped p, const double v )
            : pose( p ), speed( v ) {}
    };

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PathDriver( ros::Publisher& cmd_pub, MotionControlNode *node);
    virtual ~PathDriver();

    virtual void start();
    virtual void stop();
    virtual int getType() {
        return motion_control::MotionGoal::MOTION_FOLLOW_PATH;
    }

    virtual int execute( MotionFeedback& fb, MotionResult& result );
    virtual void configure( );
    virtual void setGoal( const motion_control::MotionGoal& goal );

protected:

    /**
     * @brief Calculate the waypoints from a given path.
     *
     * @param path The path (poses only). Should contain at least two poses.
     */
    void calculateWaypoints( const nav_msgs::Path &path );

    double calculateWaypointSpeed( const Eigen::Vector3d &prev, const Eigen::Vector3d &wp, const Eigen::Vector3d &next );

    bool calculateSpeed( const double request, const double beta );

    /**
     * @brief Predict the pose of the robot.
     * Author: bohlmann
     *
     * @param dt Time step
     * @param deltaf Front steering angle
     * @param deltar Rear steering angle
     * @param v Velocity of the robot
     * @param front_pred Predicted position of the front axis in robot coordinates.
     * @param rear_pred Predicted position of the rear axis in robot coordinates.
     */
    void predictPose( const double dt,
                      const double deltaf,
                      const double deltar,
                      const double v,
                      Vector2d &front_pred,
                      Vector2d &rear_pred );

    /**
     * @brief Publish steer/speed command
     *
     * @param cmd Steer/speed command (steer_front, steer_rear, speed).
     */
    void publishCmd( const Eigen::Vector3d &cmd );

    /**
     * @brief Convert a ROS pose to a Eigen vector.
     * @param in ROS pose.
     * @param out Output vector.
     */
    void rosToEigen( const geometry_msgs::Pose &in, Eigen::Vector3d &out );

    void drawArrow(int id, const geometry_msgs::Pose &pose, const std::string& ns, float r, float g, float b);

private:
    MotionControlNode* node_;

    /// Used to publish steering/speed commands
    ros::Publisher cmd_pub_;

    ros::Publisher vis_pub_;

    /// Used to transform waypoints into the local coordinate system
    tf::TransformListener tf_;

    /// The path
    std::vector<Waypoint> path_;

    /// Index of current waypoint
    int path_idx_;

    /// True if we are controlling the robot. False otherwise
    bool active_;

    /// -1 if there is no error. State if there is an error.
    int pending_error_;

    /// Minimum distance to the next waypoint
    double wp_tolerance_;

    /// Minimum distance to goal
    double goal_tolerance_;

    /// Controller deadtime
    double dead_time_;

    /// Effective distance between front and rear axis
    double l_;

    /// Maximum allowed speed
    double max_speed_;

    /// Minimum speed
    double min_speed_;

    /// Speed while driving backwards
    double reverse_speed_;

    /// Current speed
    double current_speed_;

    /// Last command (steer_front, steer_rear, speed)
    Eigen::Vector3d last_cmd_;

    /// Dual axis PID control
    bool front_only_;

    PidCtrl mono_ctrl_;
    DualPidCtrl dual_ctrl_;
};

} // namespace

#endif // PATHDRIVER_H
