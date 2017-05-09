/// HEADER
#include <path_follower/path_follower_server.h>

/// PROJECT
#include <path_follower/pathfollower.h>
#include <path_follower/utils/path_exceptions.h>

/// SYSTEM
#include <boost/variant.hpp>
#include <ros/ros.h>

PathFollowerServer::PathFollowerServer(PathFollower &follower)
    : follower_(follower),
      follow_path_server_(follower.getNodeHandle(), "follow_path", false)
{
    // Init. action server
    follow_path_server_.registerGoalCallback([this]() { followPathGoalCB(); });
    follow_path_server_.registerPreemptCallback([this]() {followPathPreemptCB(); });

    follow_path_server_.start();

    double continue_mode_timeout_seconds = follower.getNodeHandle().param("continue_mode_timeout_seconds", 0.1);
    continue_mode_timeout_ = ros::Duration(continue_mode_timeout_seconds);
}

void PathFollowerServer::spin()
{
    ros::Rate rate(50);
    ros::Rate idle_rate(5);

    while(ros::ok()) {
        try {
            ros::spinOnce();
            update();

            if(follow_path_server_.isActive()) {
                rate.sleep();
            } else {
                idle_rate.sleep();
            }
        } catch (const EmergencyBreakException &e) {
            ROS_ERROR("Emergency Break [status %d]: %s", e.status_code, e.what());
            follower_.emergencyStop();

            path_msgs::FollowPathResult result;
            result.status = e.status_code;
            follow_path_server_.setAborted(result);
        }
    }
}

void PathFollowerServer::update()
{
    if (follow_path_server_.isActive()) {
        if(follow_path_server_.isPreemptRequested()) {
            followPathPreemptCB();

        } else {
            auto result_var = follower_.update();

            if (result_var.type() == typeid(path_msgs::FollowPathFeedback)) {
                auto feedback = boost::get<path_msgs::FollowPathFeedback>(result_var);
                follow_path_server_.publishFeedback(feedback);
            } else {
                auto result = boost::get<path_msgs::FollowPathResult>(result_var);
                if (result.status == path_msgs::FollowPathResult::RESULT_STATUS_SUCCESS) {
                    follow_path_server_.setSucceeded(result);
                } else {
                    follow_path_server_.setAborted(result);
                }
            }
        }
    } else {
        if(last_preempt_ && ros::Time::now() > last_preempt_.get() + continue_mode_timeout_) {
            ROS_WARN_STREAM("stoping the robot in continue mode, because " << continue_mode_timeout_ << " time has passed" << std::endl);
            follower_.stop(path_msgs::FollowPathResult::RESULT_STATUS_SUCCESS);
            last_preempt_.reset();
        }
    }
}



void PathFollowerServer::followPathGoalCB()
{
    latest_goal_ = follow_path_server_.acceptNewGoal();
    ROS_INFO("Start Action!");

    // stop current goal
    if(follower_.isRunning()) {
        if(latest_goal_->follower_options.init_mode != path_msgs::FollowerOptions::INIT_MODE_CONTINUE) {
            follower_.stop(path_msgs::FollowPathResult::RESULT_STATUS_SUCCESS);
        }
    }

    follower_.setGoal(*latest_goal_);
}

void PathFollowerServer::followPathPreemptCB()
{
    if(follower_.isRunning()) {
        if(latest_goal_->follower_options.init_mode != path_msgs::FollowerOptions::INIT_MODE_CONTINUE) {
            follower_.stop(path_msgs::FollowPathResult::RESULT_STATUS_SUCCESS);
        }
    }
    follow_path_server_.setPreempted();

    last_preempt_ = ros::Time::now();
}
