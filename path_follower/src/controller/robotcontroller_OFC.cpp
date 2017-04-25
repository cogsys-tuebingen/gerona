// HEADER
#include <path_follower/controller/robotcontroller_OFC.h>

// THIRD PARTY
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


// PROJECT
#include <path_follower/pathfollowerparameters.h>
#include <path_follower/utils/cubic_spline_interpolation.h>
#include <cslibs_utils/MathHelper.h>
#include <path_follower/utils/pose_tracker.h>
#include <path_follower/utils/visualizer.h>
#include <path_follower/utils/obstacle_cloud.h>
#include <path_follower/obstacle_avoidance/obstacleavoider.h>


// SYSTEM
#include <cmath>
#include <deque>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <boost/algorithm/clamp.hpp>
#include <pcl_ros/point_cloud.h>
#include <pluginlib/class_list_macros.h>

using namespace Eigen;


RobotController_OFC::RobotController_OFC():
    RobotController(),
    cmd_(this),
    vn_(0.0),
    Ts_(0.02),
    e_prev_lin_(0.0),
    e_prev_ang_(0.0),
    e_sum_lin_(0.0),
    e_sum_ang_(0.0),
    counter_(1),
    mean_vel_(0.0)
{
    T_prev_ = ros::Time::now();
    previous_t_ = ros::Time::now();
    meas_velocities_ = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    last_movement_ = ros::Time::now();
    target_vec_prev_ = tf::Vector3(0,0,0);

    goal_pub = nh_.advertise<geometry_msgs::PointStamped>("goal_position", 0);
    target_pub = nh_.advertise<geometry_msgs::PointStamped>("target_position", 0);
    err_vec_pub = nh_.advertise<visualization_msgs::Marker>("error_vector", 0);
}

void RobotController_OFC::stopMotion()
{

    cmd_.speed = 0;
    cmd_.direction_angle = 0;
    cmd_.rotation = 0;

    MoveCommand mcmd = cmd_;
    publishMoveCommand(mcmd);
}

void RobotController_OFC::initialize()
{
    RobotController::initialize();

    // desired velocity
    vn_ = std::min(global_opt_->max_velocity(), velocity_);
    ROS_WARN_STREAM("velocity_: " << velocity_ << ", vn: " << vn_);

    //reset the PID variables
    e_sum_lin_ = 0.0;
    e_sum_ang_ = 0.0;
    T_prev_ = ros::Time::now();

}


RobotController::MoveCommandStatus RobotController_OFC::computeMoveCommand(MoveCommand *cmd)
{
    // omni drive can rotate.
    *cmd = MoveCommand(true);


    ///get the person's velocity
    tf::Transform target_tf_robot = pose_tracker_->getRelativeTransform(pose_tracker_->getRobotFrameId(), "target", ros::Time::now(), ros::Duration(0.01));
    tf::Vector3 target_vec = target_tf_robot.getOrigin();

    if(std::abs(target_vec.length() - target_vec_prev_.length()) > 1e-1){
        ros::Time current_t = ros::Time::now();
        double dt = (current_t - previous_t_).toSec();
        previous_t_ = current_t;
        tf::Vector3 target_v_vec = (target_vec - target_vec_prev_)/std::max(1e-5,dt);
        target_vec_prev_ = target_vec;
        double target_v = target_v_vec.length()  <= vn_ ? target_v_vec.length() : vn_;

        //calculate mean value using the "meas_velocities.size()" last measured velocities
        //e.g. if meas_velocities_ = {0, 0}, the mean over the last two values is used, i.e. mean_vel_ = (i[t] + i[t-1])/2.0
        meas_velocities_[counter_] = target_v;
        mean_vel_ = 0.0;
        int meas_num = meas_velocities_.size();
        for(double i : meas_velocities_){
            mean_vel_ += i;
            if(i < 1e-5){
                meas_num--;
            }
        }
        mean_vel_ /= meas_num;

        if(counter_ == meas_velocities_.size() - 1){
            counter_ = -1;
        }
        counter_++;

        last_movement_ = ros::Time::now();

    }
    ///***///

    Eigen::Vector2d rob2goal(opt_.goal_x(), opt_.goal_y());
    Eigen::Vector2d rob2target(target_vec.getX(), target_vec.getY());
    Eigen::Vector2d goal2target = rob2target - rob2goal;

    //publish the desired goal position
    geometry_msgs::PointStamped goal_pose;
    goal_pose.point.x = opt_.goal_x();
    goal_pose.point.y = opt_.goal_y();
    goal_pose.header.frame_id = pose_tracker_->getRobotFrameId();
    goal_pub.publish(goal_pose);
    //***//

    //publish the target (person) position
    geometry_msgs::PointStamped target_pose;
    target_pose.point.x = target_vec.getX();
    target_pose.point.y = target_vec.getY();
    target_pose.header.frame_id = pose_tracker_->getRobotFrameId();
    target_pub.publish(target_pose);
    //***//

    //publish the error vector
    visualization_msgs::Marker err_vec_marker;
    err_vec_marker.header.frame_id = pose_tracker_->getRobotFrameId();
    err_vec_marker.header.stamp = ros::Time();
    err_vec_marker.ns = "error_vector";
    err_vec_marker.id = 4;
    err_vec_marker.type = visualization_msgs::Marker::ARROW;
    err_vec_marker.action = visualization_msgs::Marker::ADD;

    err_vec_marker.pose.position.x = opt_.goal_x();
    err_vec_marker.pose.position.y = opt_.goal_y();
    err_vec_marker.pose.position.z = 0.0;
    tf::Quaternion quaternion = tf::createQuaternionFromYaw(std::atan2(goal2target[1], goal2target[0]));
    err_vec_marker.pose.orientation.x = quaternion.getX();
    err_vec_marker.pose.orientation.y = quaternion.getY();
    err_vec_marker.pose.orientation.z = quaternion.getZ();
    err_vec_marker.pose.orientation.w = quaternion.getW();
    err_vec_marker.scale.x = goal2target.norm();
    err_vec_marker.scale.y = 0.1f;
    err_vec_marker.scale.z = 0.1f;
    err_vec_marker.color.a = 1.0f;
    err_vec_marker.color.r = 0.0f;
    err_vec_marker.color.g = 1.0f;
    err_vec_marker.color.b = 0.0f;

    err_vec_pub.publish(err_vec_marker);
    //***//

    if(mean_vel_ < 1e-3){
        cmd_.speed = 0;
        cmd_.direction_angle = 0;
        cmd_.rotation = 0;

        *cmd = cmd_;

        double distance_to_goal_eucl = goal2target.norm();

        ROS_INFO_THROTTLE(1, "Final positioning error: %f m, %f deg", distance_to_goal_eucl, MathHelper::Angle(rob2goal, rob2target)*180./M_PI);

        return RobotController::MoveCommandStatus::REACHED_GOAL;
    }

    double Ta = std::max(1e-3, (ros::Time::now() - T_prev_).toSec());
    double e_lin = goal2target.norm();
    double e_ang = std::atan2(rob2target[1] - rob2goal[1], rob2target[0] - rob2goal[0]);
    e_sum_lin_ += e_lin*Ta;
    e_sum_ang_ += e_ang*Ta;
    double v = opt_.kp_lin()*e_lin + opt_.ki_lin()*e_sum_lin_ + opt_.kd_lin()*(e_lin - e_prev_lin_)/Ta;
    double w = opt_.kp_ang()*e_ang + opt_.ki_ang()*e_sum_ang_ + opt_.kd_ang()*(e_ang - e_prev_ang_)/Ta;
    T_prev_ = ros::Time::now();
    e_prev_lin_ = e_lin;
    e_prev_ang_ = e_ang;

    cmd_.speed = boost::algorithm::clamp(v, 0.0, global_opt_->max_velocity());
    cmd_.rotation = boost::algorithm::clamp(w, -opt_.max_angular_velocity(), opt_.max_angular_velocity());

    *cmd = cmd_;

    ///***///

    return MoveCommandStatus::OKAY;
}

void RobotController_OFC::publishMoveCommand(const MoveCommand &cmd) const
{
    geometry_msgs::Twist msg;
    msg.linear.x  = cmd.getVelocity();
    msg.linear.y  = 0;
    msg.angular.z = cmd.getRotationalVelocity();

    cmd_pub_.publish(msg);
}
