#include "MotionControlNode.h"
#include <ramaxx_msgs/RamaxxMsg.h>
#include "MotionController.h"
#include "BehaviouralPathDriver.h"

using namespace motion_control;

MotionControlNode::MotionControlNode(ros::NodeHandle& nh, const std::string& name)
    :nh_(nh),nh_private_("~"),action_server_(nh,name, false), speed_filter_( 6 ), action_name_(name)
{
    action_server_.registerGoalCallback(boost::bind(&MotionControlNode::goalCallback,this));
    action_server_.registerPreemptCallback(boost::bind(&MotionControlNode::preemptCallback,this));
    action_server_.start();

    nh_private_.param<string>("world_frame", world_frame_, "/map");
    nh_private_.param<string>("robot_frame", robot_frame_, "/base_link");
    nh_private_.param<string>("odom_topic",  odom_topic_,  "/odom");
    nh_private_.param<string>("scan_topic",  scan_topic_,  "/scan");
    nh_private_.param<string>("cmd_topic",   cmd_topic_,   "/ramaxx_cmd");

    ROS_INFO_STREAM("world_frame: " << world_frame_);
    ROS_INFO_STREAM("robot_frame: " << robot_frame_);
    ROS_INFO_STREAM("odom_topic: "  << odom_topic_);
    ROS_INFO_STREAM("scan_topic: "  << scan_topic_);

    cmd_ramaxx_pub_ = nh_.advertise<ramaxx_msgs::RamaxxMsg> (cmd_topic_, 10 );
    scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>( scan_topic_, 1, boost::bind(&MotionControlNode::laserCallback, this, _1 ));
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>( odom_topic_, 1, boost::bind( &MotionControlNode::odometryCallback, this, _1 ));
    sonar_sub_ = nh_.subscribe<sensor_msgs::PointCloud>( "/sonar_raw", 1, boost::bind( &MotionControlNode::sonarCallback, this, _1 ));
    active_ctrl_ = NULL;
    path_driver_ = new BehaviouralPathDriver(cmd_ramaxx_pub_, this);

    action_server_.start();
}


MotionControlNode::~MotionControlNode()
{

    delete path_driver_;
}


bool MotionControlNode::transformToLocal(const geometry_msgs::PoseStamped &global, Vector3d &local)
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


bool MotionControlNode::transformToLocal(const geometry_msgs::PoseStamped &global_org, geometry_msgs::PoseStamped &local)
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

bool MotionControlNode::transformToGlobal(const geometry_msgs::PoseStamped &local_org, geometry_msgs::PoseStamped &global)
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


bool MotionControlNode::getWorldPose(Vector3d &pose , geometry_msgs::Pose *pose_out) const
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


void MotionControlNode::goalCallback()
{  
    boost::shared_ptr<const motion_control::MotionGoal_<std::allocator<void> > >
            goalptr = action_server_.acceptNewGoal();
    if (active_ctrl_!=NULL/* && goalptr->mode!=active_ctrl_->getType()*/) {
        active_ctrl_->stop();
    }
    switch (goalptr->mode) {
    case motion_control::MotionGoal::MOTION_FOLLOW_PATH:
        active_ctrl_ = path_driver_;
        active_ctrl_->setGoal( *goalptr );
        break;
    default:
        ROS_WARN("Motioncontrol invalid motion mode %d requested",goalptr->mode);
        MotionResult result;
        result.status=MotionResult::MOTION_STATUS_INTERNAL_ERROR;
        action_server_.setAborted(result);
        active_ctrl_=0;
    }


}

void MotionControlNode::laserCallback(const sensor_msgs::LaserScanConstPtr& scan)
{
    if (active_ctrl_!=NULL) {
        active_ctrl_->laserCallback(scan);
    }
}

void MotionControlNode::odometryCallback( const nav_msgs::OdometryConstPtr &odom )
{
    odometry_ = *odom;
    float current_speed = sqrt( pow( odom->twist.twist.linear.x, 2 ) + pow( odom->twist.twist.linear.y, 2 ));
    speed_filter_.Update( current_speed );
}

void MotionControlNode::sonarCallback( const sensor_msgs::PointCloudConstPtr &data )
{
    if ( active_ctrl_ != NULL )
        active_ctrl_->sonarCallback( data );
}

void MotionControlNode::preemptCallback()
{
    if ( active_ctrl_ != NULL ) {
        active_ctrl_->stop(); /// @todo think about this
    }

}


void MotionControlNode::update()
{
    if (active_ctrl_!=NULL) {
        MotionFeedback feedback;
        MotionResult result;
        int status=active_ctrl_->execute(feedback,result);
        switch (status) {
        case MotionResult::MOTION_STATUS_STOP:
            // nothing to do
            break;
        case MotionResult::MOTION_STATUS_MOVING:
            action_server_.publishFeedback(feedback);
            break;
        case MotionResult::MOTION_STATUS_SUCCESS:
            result.status = MotionResult::MOTION_STATUS_SUCCESS;
            action_server_.setSucceeded( result );
            active_ctrl_ = NULL;
            break;
        case MotionResult::MOTION_STATUS_COLLISION:
        case MotionResult::MOTION_STATUS_INTERNAL_ERROR:
        case MotionResult::MOTION_STATUS_SLAM_FAIL:
        case MotionResult::MOTION_STATUS_PATH_LOST:
        default:
            result.status = status;
            action_server_.setAborted(result);
            active_ctrl_ = NULL;
            status=MotionResult::MOTION_STATUS_STOP;
            break;
        }
    }
}


int main(int argc, char** argv)
{
    ros::init(argc,argv,"motion_control");
    ros::NodeHandle nh;
    MotionControlNode node(nh,"motion_control");

    ros::Rate rate(50);

    while( ros::ok() )
    {
        ros::spinOnce();
        node.update();
        rate.sleep();
    }

    return 0;
}
