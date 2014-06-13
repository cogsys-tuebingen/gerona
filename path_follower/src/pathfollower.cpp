#include "pathfollower.h"
#include <geometry_msgs/Twist.h>
// Controller/Models
#include "robotcontroller_ackermann_pid.h"
#include "robotcontroller_omnidrive_pid.h"

using namespace path_msgs;
using namespace std;

PathFollower::PathFollower(ros::NodeHandle &nh):
    node_handle_(nh),
    follow_path_server_(nh, "follow_path", false),
    active_ctrl_(NULL),
    use_obstacle_map_(false)
{
    // Init. action server
    follow_path_server_.registerGoalCallback(boost::bind(&PathFollower::followPathGoalCB, this));
    follow_path_server_.registerPreemptCallback(boost::bind(&PathFollower::followPathPreemptCB,this));

    string param_controller;

    ros::param::param<string>("~world_frame", world_frame_, "/map");
    ros::param::param<string>("~robot_frame", robot_frame_, "/base_link");
    ros::param::param<bool>("~use_obstacle_map", use_obstacle_map_, false);
    ros::param::param<bool>("~use_vfh", use_vfh_, false);
    ros::param::param<string>("~controller", param_controller, "ackermann_pid");

    //cmd_pub_ = nh_.advertise<ramaxx_msgs::RamaxxMsg> (cmd_topic_, 10);
    std::string cmd_vel;
    ros::param::param<string>("~cmd_vel", cmd_vel, "/cmd_vel");
    cmd_pub_ = node_handle_.advertise<geometry_msgs::Twist> (cmd_vel, 10);

    speech_pub_ = node_handle_.advertise<std_msgs::String>("/speech", 0);

    odom_sub_ = node_handle_.subscribe<nav_msgs::Odometry>("/odom", 1, &PathFollower::odometryCB, this);

    if(use_obstacle_map_) {
        obstacle_map_sub_ = node_handle_.subscribe<nav_msgs::OccupancyGrid>("/obstacle_map", 0, boost::bind(&PathFollower::obstacleMapCB, this, _1));
    } else {
        laser_sub_ = node_handle_.subscribe<sensor_msgs::LaserScan>("/scan", 10, boost::bind(&PathFollower::laserCB, this, _1));
    }

    active_ctrl_ = new BehaviouralPathDriver(this);

    VectorFieldHistogram* vfh_ptr = use_vfh_ ? &vfh_ : 0;

    ROS_INFO("Use robot controller '%s'", param_controller.c_str());
    if (param_controller == "ackermann_pid") {
        //TODO: this cast can cause problems......................!!
        controller_ = new RobotController_Ackermann_Pid(cmd_pub_, (BehaviouralPathDriver*) active_ctrl_, vfh_ptr);
    } else if (param_controller == "omnidrive_pid") {
        controller_ = new RobotController_Omnidrive_Pid(cmd_pub_, (BehaviouralPathDriver*) active_ctrl_);
    } else {
        ROS_FATAL("Unknown robot controller. Shutdown.");
        exit(1);
    }

    follow_path_server_.start();
    ROS_INFO("Initialisation done.");
}

PathFollower::~PathFollower()
{
    delete controller_;
    delete active_ctrl_;
}


void PathFollower::followPathGoalCB()
{
    FollowPathGoalConstPtr goalptr = follow_path_server_.acceptNewGoal();
    ROS_INFO("Start Action!");

    // stop current goal
    active_ctrl_->stop();

    controller_->setVelocity(goalptr->velocity);
    active_ctrl_->setGoal(*goalptr);
}

void PathFollower::followPathPreemptCB()
{
    active_ctrl_->stop(); active_ctrl_->stop(); /// @todo think about this
}

void PathFollower::odometryCB(const nav_msgs::OdometryConstPtr &odom)
{
    odometry_ = *odom;
}

void PathFollower::laserCB(const sensor_msgs::LaserScanConstPtr &scan)
{
    laser_scan_ = *scan;
}

void PathFollower::obstacleMapCB(const nav_msgs::OccupancyGridConstPtr &map)
{
    controller_->getObstacleDetector()->gridMapCallback(map);

    if(use_vfh_) {
        vfh_.setMap(*map);
    }
}


bool PathFollower::getWorldPose(Vector3d *pose_vec , geometry_msgs::Pose *pose_msg) const
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

    pose_vec->x()  = msg.transform.translation.x;
    pose_vec->y()  = msg.transform.translation.y;
    (*pose_vec)(2) = tf::getYaw(msg.transform.rotation);

    if(pose_msg != NULL) {
        pose_msg->position.x = msg.transform.translation.x;
        pose_msg->position.y = msg.transform.translation.y;
        pose_msg->position.z = msg.transform.translation.z;
        pose_msg->orientation = msg.transform.rotation;
    }
    return true;
}

geometry_msgs::Twist PathFollower::getVelocity() const
{
//    geometry_msgs::Twist twist;
//    try {
//        pose_listener_.lookupTwist("/odom", robot_frame_, ros::Time(0), ros::Duration(0.01), twist);

//    } catch (tf::TransformException& ex) {
//        ROS_ERROR("error with transform robot pose: %s", ex.what());
//        return geometry_msgs::Twist();
//    }
//    return twist;
    return odometry_.twist.twist;
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
        FollowPathFeedback feedback;
        FollowPathResult result;

        int is_running = active_ctrl_->execute(feedback, result);

        if (is_running) {
            follow_path_server_.publishFeedback(feedback);
        } else {
            if (result.status == FollowPathResult::MOTION_STATUS_SUCCESS) {
                follow_path_server_.setSucceeded(result);
            } else {
                follow_path_server_.setAborted(result);
            }
        }
    }
}

bool PathFollower::checkCollision(double course_angle, double box_length, double box_width, double curve_enlarge_factor)
{
    if(use_obstacle_map_) {
        return controller_->getObstacleDetector()->isObstacleAhead(box_width, box_length, course_angle, curve_enlarge_factor);
    } else {
        return laser_env_.CheckCollision(laser_scan_.ranges,laser_scan_.angle_min,laser_scan_.angle_max, course_angle,
                                         box_width, curve_enlarge_factor, box_length);
    }
}

bool PathFollower::simpleCheckCollision(float box_width, float box_length)
{
    for (size_t i=0; i < laser_scan_.ranges.size(); ++i) {
        // project point to carthesian coordinates
        float angle = laser_scan_.angle_min + i * laser_scan_.angle_increment;
        float px = laser_scan_.ranges[i] * cos(angle);
        float py = laser_scan_.ranges[i] * sin(angle);


        /* Point p is inside the rectangle, if
         *    p.x in [-width/2, +width/2]
         * and
         *    p.y in [0, length]
         */

        if ( py >= -box_width/2 &&
             py <=  box_width/2 &&
             px >= 0 &&
             px <= box_length )
        {
            return true;
        }
    }

//    //visualize box
//    geometry_msgs::Point p1, p2, p3, p4;
//    p1.y = -box_width/2;  p1.x = 0;
//    p2.y = -box_width/2;  p2.x = box_length;
//    p3.y = +box_width/2;  p3.x = 0;
//    p4.y = +box_width/2;  p4.x = box_length;

//    float r = collision ? 1 : 0;
//    float g = 1 - r;
//    visualizer_->drawLine(1, p1, p2, "laser", "collision_box", r,g,0, 3, 0.05);
//    visualizer_->drawLine(2, p2, p4, "laser", "collision_box", r,g,0, 3, 0.05);
//    visualizer_->drawLine(3, p1, p3, "laser", "collision_box", r,g,0, 3, 0.05);
//    visualizer_->drawLine(4, p3, p4, "laser", "collision_box", r,g,0, 3, 0.05);

    return false;
}

VectorFieldHistogram& PathFollower::getVFH()
{
    return vfh_;
}

RobotController *PathFollower::getController()
{
    return controller_;
}

void PathFollower::say(string text)
{
    std_msgs::String str;
    str.data = text;
    speech_pub_.publish(str);
}
