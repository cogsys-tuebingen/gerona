#include "pathfollower.h"
#include <geometry_msgs/Twist.h>

PathFollower::PathFollower(ros::NodeHandle &nh):
    node_handle_(nh),
    follow_path_server_(nh, "follow_path", false),
    active_ctrl_(NULL)
{
    // Init. action server
    follow_path_server_.registerGoalCallback(boost::bind(&PathFollower::followPathGoalCB, this));
    follow_path_server_.registerPreemptCallback(boost::bind(&PathFollower::followPathPreemptCB,this));

    ros::param::param<string>("~world_frame", world_frame_, "/map");
    ros::param::param<string>("~robot_frame", robot_frame_, "/base_link");
    ros::param::param<float>("~stuck_timeout", opt_.stuck_timeout, 10); //TODO: find reasonable default value
    ros::param::param<float>("~stuck_pos_tolerance", opt_.stuck_pos_tolerance, 0.1); //TODO: find reasonable default value

    //cmd_pub_ = nh_.advertise<ramaxx_msgs::RamaxxMsg> (cmd_topic_, 10);
    cmd_pub_ = node_handle_.advertise<geometry_msgs::Twist> ("/cmd_vel", 10);

    odom_sub_ = node_handle_.subscribe<nav_msgs::Odometry>("/odom", 1, &PathFollower::odometryCB, this);

    stuck_timeout_ = new StuckTimeout(node_handle_, opt_.stuck_timeout, opt_.stuck_pos_tolerance);

    active_ctrl_ = new motion_control::BehaviouralPathDriver(cmd_pub_, this);

    follow_path_server_.start();
    ROS_INFO("Initialisation done.");
}

PathFollower::~PathFollower()
{
    delete active_ctrl_;
    delete stuck_timeout_;
}



void PathFollower::followPathGoalCB()
{
    path_msgs::FollowPathGoalConstPtr goalptr = follow_path_server_.acceptNewGoal();
    ROS_INFO("Start Action!! [%d]", goalptr->debug_test);

    // stop current goal
    active_ctrl_->stop();

    active_ctrl_->setGoal(*goalptr);
    stuck_timeout_->start();
}

void PathFollower::followPathPreemptCB()
{
    active_ctrl_->stop(); active_ctrl_->stop(); /// @todo think about this
    stuck_timeout_->stop();
}

void PathFollower::odometryCB(const nav_msgs::OdometryConstPtr &odom)
{
    odometry_ = *odom;
}

bool PathFollower::getWorldPose(Vector3d &pose , geometry_msgs::Pose *pose_out) const
{
    tf::StampedTransform transform;
    geometry_msgs::TransformStamped msg;

    try {
        pose_listener_.lookupTransform(world_frame_, robot_frame_, ros::Time(0), transform);
    } catch (tf::TransformException& ex) {
        ROS_ERROR("error with transform robot pose: %s", ex.what());
        return false;
    }
    tf::transformStampedTFToMsg(transform, msg);

    pose.x() = msg.transform.translation.x;
    pose.y() = msg.transform.translation.y;
    pose(2) = tf::getYaw(msg.transform.rotation);

    if(pose_out != NULL) {
        pose_out->position.x = msg.transform.translation.x;
        pose_out->position.y = msg.transform.translation.y;
        pose_out->position.z = msg.transform.translation.z;
        pose_out->orientation = msg.transform.rotation;
    }
    return true;
}

bool PathFollower::transformToLocal(const geometry_msgs::PoseStamped &global, Vector3d &local)
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


bool PathFollower::transformToLocal(const geometry_msgs::PoseStamped &global_org, geometry_msgs::PoseStamped &local)
{
    geometry_msgs::PoseStamped global(global_org);
    try {
        global.header.frame_id=world_frame_;
        pose_listener_.transformPose(robot_frame_,ros::Time(0),global,world_frame_,local);
        return true;

    } catch (tf::TransformException& ex) {
        ROS_ERROR("error with transform goal pose: %s", ex.what());
        return false;
    }
}

bool PathFollower::transformToGlobal(const geometry_msgs::PoseStamped &local_org, geometry_msgs::PoseStamped &global)
{
    geometry_msgs::PoseStamped local(local_org);
    try {
        local.header.frame_id=robot_frame_;
        pose_listener_.transformPose(world_frame_,ros::Time(0),local,robot_frame_,global);
        return true;

    } catch (tf::TransformException& ex) {
        ROS_ERROR("error with transform goal pose: %s", ex.what());
        return false;
    }
}

void PathFollower::update()
{
    //TODO: is isActive() good here?
    if (follow_path_server_.isActive() && active_ctrl_!=NULL) {
        path_msgs::FollowPathFeedback feedback;
        path_msgs::FollowPathResult result;
        bool done = false;

        int status = active_ctrl_->execute(feedback, result);

        // If the robot is stuck (timeout expired), change status to MOVE_FAIL, no matter what active_ctrl is saying.
        if (stuck_timeout_->isStuck()) { //TODO: use special status for this
            status = path_msgs::FollowPathResult::MOTION_STATUS_MOVE_FAIL;
            ROS_WARN("Robot is stuck");
        }

        switch (status) {
        case path_msgs::FollowPathResult::MOTION_STATUS_STOP:
            // nothing to do
            break;

        case path_msgs::FollowPathResult::MOTION_STATUS_MOVING:
            follow_path_server_.publishFeedback(feedback);
            break;

        case path_msgs::FollowPathResult::MOTION_STATUS_SUCCESS:
            result.status = path_msgs::FollowPathResult::MOTION_STATUS_SUCCESS;
            follow_path_server_.setSucceeded(result);
            done = true;
            //active_ctrl_ = NULL;
            break;

        case path_msgs::FollowPathResult::MOTION_STATUS_COLLISION:
        case path_msgs::FollowPathResult::MOTION_STATUS_INTERNAL_ERROR:
        case path_msgs::FollowPathResult::MOTION_STATUS_SLAM_FAIL:
        case path_msgs::FollowPathResult::MOTION_STATUS_PATH_LOST:
        default:
            result.status = status;
            follow_path_server_.setAborted(result);
            done = true;
            //active_ctrl_ = NULL;
            break;
        }

        if (done) {
            // stop timer while not needed to avoid unnecessary CPU load.
            stuck_timeout_->stop();
        }
    }
}
