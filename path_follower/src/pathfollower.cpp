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

    // TODO: drop this params?
    ros::param::param<string>("~world_frame", world_frame_, "/map");
    ros::param::param<string>("~robot_frame", robot_frame_, "/base_link");
    // Don't use params, there are remappings for that
    //ros::param::param<string>("~odom_topic",  odom_topic_,  "/odom");
    //ros::param::param<string>("~scan_topic",  scan_topic_,  "/scan");
    //ros::param::param<string>("~cmd_topic",   cmd_topic_,   "/ramaxx_cmd");

    //cmd_pub_ = nh_.advertise<ramaxx_msgs::RamaxxMsg> (cmd_topic_, 10);
    cmd_pub_ = node_handle_.advertise<geometry_msgs::Twist> ("/cmd_vel", 10);

    //FIXME: are laser and sonar required here?
    //scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>( scan_topic_, 1, boost::bind(&MotionControlNode::laserCallback, this, _1 ));
    //sonar_sub_ = nh_.subscribe<sensor_msgs::PointCloud>( "/sonar_raw", 1, boost::bind( &MotionControlNode::sonarCallback, this, _1 ));

    odom_sub_ = node_handle_.subscribe<nav_msgs::Odometry>("/odom", 1, boost::bind(&PathFollower::odometryCB, this, _1 ));

    active_ctrl_ = new motion_control::BehaviouralPathDriver(cmd_pub_, this);

    follow_path_server_.start();
    ROS_INFO("Initialisation done.");
}

PathFollower::~PathFollower()
{
    delete active_ctrl_;
}



void PathFollower::followPathGoalCB()
{

    // dummy feedback
    /*
    path_msgs::FollowPathFeedback feed;
    feed.debug_test = goal->debug_test;

    for (int i = 0; i<6; ++i) {
        ros::Duration(1).sleep();

        if (follow_path_server_.isPreemptRequested()) {
            ROS_INFO("Preemt goal [%d].\n---------------------", goal->debug_test);
            path_msgs::FollowPathResult res;
            res.status = res.STATUS_ABORTED;
            res.debug_test = goal->debug_test;
            follow_path_server_.setPreempted(res);
            return;
        }

        ROS_INFO("Feedback [%d]", goal->debug_test);
        follow_path_server_.publishFeedback(feed);
    }

    ros::Duration(3).sleep();

    path_msgs::FollowPathResult res;
    res.status = res.STATUS_SUCCESS;
    res.debug_test = goal->debug_test;
    ROS_INFO("Finished [%d].\n---------------------", goal->debug_test);
    follow_path_server_.setSucceeded(res);
    */

    path_msgs::FollowPathGoalConstPtr goalptr = follow_path_server_.acceptNewGoal();
    ROS_INFO("Start Action!! [%d]", goalptr->debug_test);

    // stop current goal
    active_ctrl_->stop();

    active_ctrl_->setGoal(*goalptr);
}

void PathFollower::followPathPreemptCB()
{
    active_ctrl_->stop(); active_ctrl_->stop(); /// @todo think about this
}

void PathFollower::odometryCB(const nav_msgs::OdometryConstPtr &odom)
{
    odometry_ = *odom;
    //float current_speed = sqrt( pow( odom->twist.twist.linear.x, 2 ) + pow( odom->twist.twist.linear.y, 2 ) );
    //speed_filter_.Update(current_speed);
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

        int status = active_ctrl_->execute(feedback, result);

        switch (status) {
        case path_msgs::FollowPathResult::MOTION_STATUS_STOP:
            // nothing to do
            break;

        case path_msgs::FollowPathResult::MOTION_STATUS_MOVING:
            follow_path_server_.publishFeedback(feedback);
            break;

        case path_msgs::FollowPathResult::MOTION_STATUS_SUCCESS:
            result.status = path_msgs::FollowPathResult::MOTION_STATUS_SUCCESS;
            follow_path_server_.setSucceeded( result );
            //active_ctrl_ = NULL;
            break;

        case path_msgs::FollowPathResult::MOTION_STATUS_COLLISION:
        case path_msgs::FollowPathResult::MOTION_STATUS_INTERNAL_ERROR:
        case path_msgs::FollowPathResult::MOTION_STATUS_SLAM_FAIL:
        case path_msgs::FollowPathResult::MOTION_STATUS_PATH_LOST:
        default:
            result.status = status;
            follow_path_server_.setAborted(result);
            //active_ctrl_ = NULL;
            //status=path_msgs::FollowPathResult::MOTION_STATUS_STOP;
            break;
        }
    }
}
