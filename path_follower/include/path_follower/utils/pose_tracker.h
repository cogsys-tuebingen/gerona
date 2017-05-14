#ifndef POSE_TRACKER_H
#define POSE_TRACKER_H

/// SYSTEM
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>

class PathFollowerParameters;

/**
 * @brief The PoseTracker class simplifies access to lookup various transformations.
 */
class PoseTracker
{
public:
    PoseTracker(const PathFollowerParameters& opt, ros::NodeHandle& nh);

    /**
     * @brief isLocal returns whether a local planner is used
     * @return true iff a local planner is used.
     */
    bool isLocal() const;

    /**
     * @brief setLocal set whether a local planner is used
     * @param local true iff a local planner is used.
     */
    void setLocal(bool local);

    /**
     * @brief getFixedFrameId returns the world frame, of no local planenr is used.
     *         Otherwise the odom frame is returned.
     * @return the name of the fixed frame
     */
    std::string getFixedFrameId() const;

    /**
     * @brief getRobotFrameId
     * @return  the name of the robot frame
     */
    std::string getRobotFrameId() const;

    /**
     * @brief getRobotPose returns the world pose of the robot, of no local planenr is used.
     *         Otherwise the odom pose is returned.
     * @return The pose of the robot in the fixed frame
     */
    Eigen::Vector3d getRobotPose() const;

    /**
     * @brief getRobotPoseMsg returns the world pose of the robot, of no local planenr is used.
     *         Otherwise the odom pose is returned.
     * @return The pose of the robot in the fixed frame
     */
    const geometry_msgs::Pose &getRobotPoseMsg() const;

    /**
     * @brief getVelocity
     * @return the velocity of the robot according to odometry
     */
    geometry_msgs::Twist getVelocity() const;

    /**
     * @brief getTransform returns the transformation between the two given frames at time <time>.
     *        If the transformation is not availible at time <time>, the latest transform will be returned.
     *        If the transformation is not available at all, an std::runtime_error is thrown.
     * @param fixed_frame Name of the parent frame
     * @param frame Name of the child frame
     * @param time Timat for which the tranform is requested
     * @param max_wait Duration in seconds to wait for the transform
     * @return  The transform at time <time> if possible, otherwise at time 0.
     * @throws std::runtime_error if the transform is not availble at all
     */
    tf::Transform getTransform(const std::string& fixed_frame, const std::string& frame, const ros::Time& time, const ros::Duration &max_wait) const;
    /**
     * @brief getRelativeTransform returns the transformation between the robot frame and the given frame at time <time>.
     *        If the transformation is not availible at time <time>, the latest transform will be returned.
     *        If the transformation is not available at all, an std::runtime_error is thrown.
     * @param frame Name of the child frame
     * @param time Timat for which the tranform is requested
     * @param max_wait Duration in seconds to wait for the transform
     * @return  The transform at time <time> if possible, otherwise at time 0.
     * @throws std::runtime_error if the transform is not availble at all
     */
    tf::Transform getRelativeTransform(const std::string& frame, const ros::Time& time, const ros::Duration &max_wait) const;

    /**
     * @brief transformToLocal transforms the world pose <global> into a relative pose <local>
     * @param global the global pose
     * @param local [out] the transformed pose <global>
     * @return true, iff <local> is a valid transform
     */
    bool transformToLocal(const geometry_msgs::PoseStamped& global, geometry_msgs::PoseStamped& local );

    /**
     * @brief transformToLocal transforms the world pose <global> into a relative pose <local>
     * @param global the global pose
     * @param local [out] the transformed pose <global>
     * @return true, iff <local> is a valid transform
     */
    bool transformToLocal(const geometry_msgs::PoseStamped& global, Eigen::Vector3d& local );

    /**
     * @brief transformToGlobal transforms the relative pose <local> into a world pose <global>
     * @param local the local pose
     * @param global [out] the transformed pose <local>
     * @return true, iff <global> is a valid transform
     */
    bool transformToGlobal(const geometry_msgs::PoseStamped& local, geometry_msgs::PoseStamped& global );

    /**
     * @brief updateRobotPose refreshed the current robot pose.
     *        This method has to be called periodically by the main thread.
     * @return true, iff the pose is available
     */
    bool updateRobotPose();

    /**
     * @brief getTransformListener accesses the underlying tf::TransformListener.
     * @return the underlying tf::TransformListener
     */
    tf::TransformListener& getTransformListener();

private:
    //! Callback for odometry messages
    void odometryCB(const nav_msgs::OdometryConstPtr &odom);

    //! Update the current pose of the robot.
    /** @see robot_pose_, robot_pose_msg_ */

    bool getWorldPose(Eigen::Vector3d *pose_vec, geometry_msgs::Pose* pose_msg = nullptr) const;

private:
    const PathFollowerParameters& opt_;
    tf::TransformListener pose_listener_;

    //! Subscriber for odometry messages.
    ros::Subscriber odom_sub_;

    //! The last received odometry message.
    nav_msgs::Odometry odometry_;

    //! Current pose of the robot as Eigen vector (x,y,theta).
    Eigen::Vector3d robot_pose_world_;
    Eigen::Vector3d robot_pose_odom_;

    //! Current pose of the robot as geometry_msgs pose.
    geometry_msgs::Pose robot_pose_world_msg_;
    geometry_msgs::Pose robot_pose_odom_msg_;

    bool local_;
};

#endif // POSE_TRACKER_H
