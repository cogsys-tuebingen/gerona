/// HEADER
#include <path_follower/utils/pose_tracker.h>

/// PROJECT
#include <path_follower/parameters/path_follower_parameters.h>

using namespace Eigen;

PoseTracker::PoseTracker(PathFollowerParameters &opt, ros::NodeHandle& nh)
    : opt_(opt),
      local_(false)
{
    odom_sub_ = nh.subscribe<nav_msgs::Odometry>("/odom", 1, &PoseTracker::odometryCB, this);
}

bool PoseTracker::isLocal() const
{
    return local_;
}

void PoseTracker::setLocal(bool local)
{
    local_ = local;
}

std::string PoseTracker::getFixedFrameId() const
{
    if(!local_) {
        return opt_.world_frame();
    } else {
        return opt_.odom_frame();
    }
}

std::string PoseTracker::getRobotFrameId() const
{
    return "base_link";
}


void PoseTracker::odometryCB(const nav_msgs::OdometryConstPtr &odom)
{
    odometry_ = *odom;

    robot_pose_odom_msg_ = odometry_.pose.pose;

    robot_pose_odom_.x() = robot_pose_odom_msg_.position.x;
    robot_pose_odom_.y() = robot_pose_odom_msg_.position.y;
    robot_pose_odom_.z() = tf::getYaw(robot_pose_odom_msg_.orientation);
}


bool PoseTracker::updateRobotPose()
{
    if (getWorldPose(&robot_pose_world_, &robot_pose_world_msg_)) {
        return true;
    } else {
        return false;
    }
}

tf::TransformListener& PoseTracker::getTransformListener()
{
    return pose_listener_;
}

bool PoseTracker::getWorldPose(Vector3d *pose_vec , geometry_msgs::Pose *pose_msg) const
{
    tf::StampedTransform transform;
    geometry_msgs::TransformStamped msg;

    try {
        pose_listener_.lookupTransform(opt_.world_frame(), opt_.robot_frame(), ros::Time(0), transform);

    } catch (tf::TransformException& ex) {
        ROS_ERROR("error with transform robot pose: %s", ex.what());
        return false;
    }
    tf::transformStampedTFToMsg(transform, msg);

    pose_vec->x()  = msg.transform.translation.x;
    pose_vec->y()  = msg.transform.translation.y;
    (*pose_vec)(2) = tf::getYaw(msg.transform.rotation);

    if(pose_msg != nullptr) {
        pose_msg->position.x = msg.transform.translation.x;
        pose_msg->position.y = msg.transform.translation.y;
        pose_msg->position.z = msg.transform.translation.z;
        pose_msg->orientation = msg.transform.rotation;
    }
    return true;
}

bool PoseTracker::transformToLocal(const geometry_msgs::PoseStamped &global, Vector3d &local)
{
    geometry_msgs::PoseStamped local_pose;
    bool status=transformToLocal(global,local_pose);
    if (!status) {
        local.x()=local.y()=local.z()=0.0;
        return false;
    }
    local.x()=local_pose.pose.position.x;
    local.y()=local_pose.pose.position.y;
    local.z()=tf::getYaw(local_pose.pose.orientation);
    return true;
}


bool PoseTracker::transformToLocal(const geometry_msgs::PoseStamped &global_org, geometry_msgs::PoseStamped &local)
{
    geometry_msgs::PoseStamped global(global_org);
    try {
        global.header.frame_id = getFixedFrameId();
        pose_listener_.transformPose(opt_.robot_frame(),ros::Time(0),global, global.header.frame_id,local);
        return true;

    } catch (tf::TransformException& ex) {
        ROS_ERROR("error with transform goal pose: %s", ex.what());
        return false;
    }
}

bool PoseTracker::transformToGlobal(const geometry_msgs::PoseStamped &local_org, geometry_msgs::PoseStamped &global)
{
    geometry_msgs::PoseStamped local(local_org);
    try {
        local.header.frame_id=opt_.robot_frame();
        pose_listener_.transformPose(getFixedFrameId(),ros::Time(0),local,opt_.robot_frame(),global);
        return true;

    } catch (tf::TransformException& ex) {
        ROS_ERROR("error with transform goal pose: %s", ex.what());
        return false;
    }
}

geometry_msgs::Twist PoseTracker::getVelocity() const
{
    //    geometry_msgs::Twist twist;
    //    try {
    //        pose_listener_.lookupTwist("odom", robot_frame_, ros::Time(0), ros::Duration(0.01), twist);

    //    } catch (tf::TransformException& ex) {
    //        ROS_ERROR("error with transform robot pose: %s", ex.what());
    //        return geometry_msgs::Twist();
    //    }
    //    return twist;
    return odometry_.twist.twist;
}

Eigen::Vector3d PoseTracker::getRobotPose() const
{
    if(!local_) {
        return robot_pose_world_;
    } else {
        return robot_pose_odom_;
    }
}

const geometry_msgs::Pose &PoseTracker::getRobotPoseMsg() const
{
    if(!local_) {
        return robot_pose_world_msg_;
    } else {
        return robot_pose_odom_msg_;
    }
}

tf::Transform PoseTracker::getRelativeTransform(const std::string &frame, const ros::Time &time, const ros::Duration& max_wait) const
{
    return getRelativeTransform(getRobotFrameId(), frame, time, max_wait);
}

tf::Transform PoseTracker::getRelativeTransform(const std::string &fixed_frame, const std::string &frame, const ros::Time &time, const ros::Duration& max_wait) const
{
    tf::StampedTransform trafo;
    if(pose_listener_.waitForTransform(fixed_frame, frame, time, max_wait)) {
        pose_listener_.lookupTransform(fixed_frame, frame, time, trafo);

    } else {
        ROS_WARN_STREAM_THROTTLE(0.1, "cannot lookup relative transform from " << fixed_frame << " to " << frame << " at time " << time
                        << ". Using latest transform");
        pose_listener_.lookupTransform(fixed_frame, frame, ros::Time(0), trafo);
    }
    return trafo;
}
