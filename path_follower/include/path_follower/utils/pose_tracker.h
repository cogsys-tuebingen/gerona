#ifndef POSE_TRACKER_H
#define POSE_TRACKER_H

/// SYSTEM
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>

class PathFollowerParameters;

class PoseTracker
{
public:
    PoseTracker(const PathFollowerParameters& opt, ros::NodeHandle& nh);

    bool isLocal() const;
    void setLocal(bool local);

    std::string getFixedFrameId() const;
    std::string getRobotFrameId() const;

    Eigen::Vector3d getRobotPose() const;
    const geometry_msgs::Pose &getRobotPoseMsg() const;

    geometry_msgs::Twist getVelocity() const;

    tf::Transform getRelativeTransform(const std::string& fixed_frame, const std::string& frame, const ros::Time& time, const ros::Duration &max_wait) const;
    tf::Transform getRelativeTransform(const std::string& frame, const ros::Time& time, const ros::Duration &max_wait) const;

    bool transformToLocal(const geometry_msgs::PoseStamped& global, geometry_msgs::PoseStamped& local );
    bool transformToLocal(const geometry_msgs::PoseStamped& global, Eigen::Vector3d& local );
    bool transformToGlobal(const geometry_msgs::PoseStamped& local, geometry_msgs::PoseStamped& global );

    bool updateRobotPose();

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
