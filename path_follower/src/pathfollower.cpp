#include "pathfollower.h"

/// SYSTEM
#include <boost/foreach.hpp>
#include <Eigen/Core>
#include <utils_general/MathHelper.h>
#include <cmath>
#include <cxxabi.h>
#include <boost/assign.hpp>

/// ROS
#include <geometry_msgs/Twist.h>

/// PROJECT
#include "behaviours.h"
#include <std_msgs/Int32MultiArray.h>
// Controller/Models
#include "robotcontroller_ackermann_pid.h"
#include "robotcontroller_omnidrive_pid.h"
#include "robotcontroller_omnidrive_vv.h"
#include "robotcontroller_omnidrive_orthexp.h"

using namespace path_msgs;
using namespace std;
using namespace Eigen;


namespace beep {
static std::vector<int> OBSTACLE_IN_PATH = boost::assign::list_of(25)(25)(25);
}

namespace {
std::string name(Behaviour* b) {
    int status;
    return abi::__cxa_demangle(typeid(*b).name(),  0, 0, &status);
}
}


PathFollower::PathFollower(ros::NodeHandle &nh):
    path_lookout_(this),
    node_handle_(nh),
    follow_path_server_(nh, "follow_path", false),
    use_obstacle_map_(false),
    active_behaviour_(NULL),
    pending_error_(-1)
{
    // Init. action server
    follow_path_server_.registerGoalCallback(boost::bind(&PathFollower::followPathGoalCB, this));
    follow_path_server_.registerPreemptCallback(boost::bind(&PathFollower::followPathPreemptCB,this));

    string param_controller;

    //TODO: move this to configure()
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

    VectorFieldHistogram* vfh_ptr = use_vfh_ ? &vfh_ : 0;

    ROS_INFO("Use robot controller '%s'", param_controller.c_str());
    if (param_controller == "ackermann_pid") {
        controller_ = new RobotController_Ackermann_Pid(cmd_pub_, this, vfh_ptr);
    } else if (param_controller == "omnidrive_pid") {
        controller_ = new RobotController_Omnidrive_Pid(cmd_pub_, this);
    } else if (param_controller == "omnidrive_vv") {
        controller_ = new RobotController_Omnidrive_VirtualVehicle(cmd_pub_, this);
    } else if (param_controller == "omnidrive_orthexp") {
        controller_ = new RobotController_Omnidrive_OrthogonalExponential(cmd_pub_, this);
    } else {
        ROS_FATAL("Unknown robot controller. Shutdown.");
        exit(1);
    }


    //---PathDriver
    last_beep_ = ros::Time::now();
    beep_pause_ = ros::Duration(2.0);

    visualizer_ = Visualizer::getInstance();

    beeper_ = node_handle_.advertise<std_msgs::Int32MultiArray>("/cmd_beep", 100);
    configure();
    //#

    follow_path_server_.start();
    ROS_INFO("Initialisation done.");
}



PathFollower::~PathFollower()
{
    delete controller_;
}


void PathFollower::followPathGoalCB()
{
    FollowPathGoalConstPtr goalptr = follow_path_server_.acceptNewGoal();
    ROS_INFO("Start Action!");

    // stop current goal
    stop();

    controller_->setVelocity(goalptr->velocity);
    setGoal(*goalptr);

    // don't track obstacles of former paths.
    path_lookout_.reset();
}

void PathFollower::followPathPreemptCB()
{
    stop(); stop(); /// @todo think about this
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
    controller_->getObstacleDetector()->setMap(map);
    path_lookout_.setMap(map);

    if(use_vfh_) {
        vfh_.setMap(*map);
    }
}

bool PathFollower::updateRobotPose()
{
    return getWorldPose( &robot_pose_, &robot_pose_msg_ );
}


bool PathFollower::isObstacleInBox(double course_angle, double box_length, double box_width, double curve_enlarge_factor)
{
    if(use_obstacle_map_) {
        return controller_->getObstacleDetector()->isObstacleAhead(box_width, box_length, course_angle, curve_enlarge_factor);
    } else {
        return laser_env_.CheckCollision(laser_scan_.ranges,laser_scan_.angle_min,laser_scan_.angle_max, course_angle,
                                         box_width, curve_enlarge_factor, box_length);
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

void PathFollower::spin()
{
    ros::Rate rate(50);

    while(ros::ok()) {
        ros::spinOnce();
        update();
        rate.sleep();
    }
}

void PathFollower::update()
{
    if (follow_path_server_.isActive()) {
        FollowPathFeedback feedback;
        FollowPathResult result;
        bool is_running;

        is_running = false;
        if (!updateRobotPose()) {
            result.status = FollowPathResult::MOTION_STATUS_SLAM_FAIL;
        }
        //TODO: is this a good place to run the obstacle lookout?
        else if (path_lookout_.lookForObstacles(&feedback)) {
            result.status = FollowPathResult::MOTION_STATUS_COLLISION;
            // there's an obstacle ahead, pull the emergency break!
            controller_->stopMotion();
        }
        else {
            is_running = execute(feedback, result);
        }


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

bool PathFollower::checkCollision(double course)
{
    //! Factor which defines, how much the box is enlarged in curves.
    const float enlarge_factor = 0.5; //TODO: should this be a parameter?

    /* Calculate length of the collision box, depending on current velocity.
     * v <= v_min:
     *   length = min_length
     * v > v_min && v < v_sat:
     *   length  interpolated between min_length and max_length:
     *   length = min_length + FACTOR * (max_length - min_length) * (v - v_min) / (v_sat - v_min)
     * v >= v_sat:
     *   length = max_length
     */
    float v = getVelocity().linear.x;//current_command_.velocity;

    const float diff_to_min_velocity = v - opt_.min_velocity_;

    const float norm = opt_.collision_box_velocity_saturation_ - opt_.min_velocity_;
    const float span = opt_.collision_box_max_length_ - opt_.collision_box_min_length_;
    const float interp = std::max(0.0f, diff_to_min_velocity) / std::max(norm, 0.001f);
    const float f = std::min(1.0f, opt_.collision_box_velocity_factor_ * interp);

    float box_length = opt_.collision_box_min_length_ + span * f;

    Path& current_path = paths_[opt_.path_idx];
    double distance_to_goal = current_path.back().distanceTo(current_path[opt_.wp_idx]);

    if(box_length > distance_to_goal) {
        box_length = distance_to_goal + 0.2;
    }

    if(box_length < opt_.collision_box_crit_length_) {
        box_length = opt_.collision_box_crit_length_;
    }


    bool collision = isObstacleInBox(course, box_length, opt_.collision_box_width_, enlarge_factor);

    if(collision) {
        beep(beep::OBSTACLE_IN_PATH);
    }

    return collision;
}

VectorFieldHistogram& PathFollower::getVFH()
{
    return vfh_;
}

RobotController *PathFollower::getController()
{
    return controller_;
}

PathLookout *PathFollower::getPathLookout()
{
    return &path_lookout_;
}

void PathFollower::say(string text)
{
    std_msgs::String str;
    str.data = text;
    speech_pub_.publish(str);
}

Eigen::Vector3d PathFollower::getRobotPose() const
{
    return robot_pose_;
}

const geometry_msgs::Pose &PathFollower::getRobotPoseMsg() const
{
    return robot_pose_msg_;
}


void PathFollower::start()
{
    opt_.reset();

    clearActive();

    active_behaviour_ = new BehaviourOnLine(*this);
    ROS_INFO_STREAM("init with " << name(active_behaviour_));
}

void PathFollower::stop()
{
    clearActive();

    current_command_.velocity = 0; //TODO: should be published immediately
}


bool PathFollower::execute(FollowPathFeedback& feedback, FollowPathResult& result)
{
    /* TODO:
     * The global use of the result-constants as status codes is a bit problematic, as there are feedback
     * states, which do not imply that the path execution is finished (and thus there is no result to send).
     * This is currently the case for collisions.
     */

    // constants for return codes
    const bool DONE   = false;
    const bool MOVING = true;

    // Pending error?
    if ( pending_error_ >= 0 ) {
        result.status = pending_error_;
        pending_error_ = -1;
        stop();

        return DONE;
    }

    if(paths_.empty()) {
        clearActive();
        result.status = FollowPathResult::MOTION_STATUS_SUCCESS;
        return DONE;
    }

    if(active_behaviour_ == NULL) {
        start();
    }

    visualizer_->drawArrow(0, getRobotPoseMsg(), "slam pose", 2.0, 0.7, 1.0);


    int status = FollowPathResult::MOTION_STATUS_INTERNAL_ERROR;
    try {
        ROS_DEBUG_STREAM("executing " << name(active_behaviour_));
        active_behaviour_->execute(&status);

    } catch(NullBehaviour* null) {
        ROS_WARN_STREAM("stopping after " << name(active_behaviour_));
        clearActive();

        assert(status == FollowPathResult::MOTION_STATUS_SUCCESS);

        current_command_.velocity = 0;

    } catch(Behaviour* next_behaviour) {
        std::cout << "switching behaviour from " << name(active_behaviour_) << " to " << name(next_behaviour) << std::endl;
        clearActive();
        active_behaviour_ = next_behaviour;
    }

    getController()->publishCommand();

    if(status == FollowPathResult::MOTION_STATUS_COLLISION) {
        // collision is not aborting (the obstacle might be moving away)
        feedback.status = FollowPathFeedback::MOTION_STATUS_COLLISION;
        return MOVING;
    } else if (status == FollowPathResult::MOTION_STATUS_MOVING) {
        feedback.status = FollowPathFeedback::MOTION_STATUS_MOVING;
        return MOVING;
    } else if (active_behaviour_ != NULL) {
        ROS_INFO_STREAM("aborting, clearing active, status=" << status);
        clearActive();

        result.status = status;
        return DONE;
    }
    //else
    // I think status == FollowPathResult::MOTION_STATUS_SUCCESS should be the only possible case here.
    if (status != FollowPathResult::MOTION_STATUS_SUCCESS) ROS_ERROR("I thought wrong... File: %s, Line: %d", __FILE__, __LINE__);

    result.status = status;
    return DONE;
}

void PathFollower::configure()
{
    ros::NodeHandle nh("~");
    nh.param( "waypoint_tolerance", opt_.wp_tolerance_, 0.20 );
    nh.param( "goal_tolerance", opt_.goal_tolerance_, 0.15 );
    nh.param( "steer_slow_threshold", opt_.steer_slow_threshold_, 0.25 );
    nh.param( "max_distance_to_path", opt_.max_distance_to_path_, 0.3 ); //TODO: find reasonable default value.

    // use ros::param here, because nh.param can't handle floats...
    ros::param::param<float>( "~min_velocity", opt_.min_velocity_, 0.4 );
    ros::param::param<float>( "~max_velocity", opt_.max_velocity_, 2.0 );
    ros::param::param<float>( "~collision_box_width", opt_.collision_box_width_, 0.5);
    ros::param::param<float>( "~collision_box_min_length", opt_.collision_box_min_length_, 0.8);
    ros::param::param<float>( "~collision_box_crit_length", opt_.collision_box_crit_length_, 0.3);
    ros::param::param<float>( "~collision_box_max_length", opt_.collision_box_max_length_, 1.0);
    ros::param::param<float>( "~collision_box_velocity_factor", opt_.collision_box_velocity_factor_, 1.0);
    ros::param::param<float>( "~collision_box_velocity_saturation", opt_.collision_box_velocity_saturation_, opt_.max_velocity_);

    if(opt_.max_velocity_ < opt_.min_velocity_) {
        ROS_ERROR("min velocity larger than max velocity!");
        opt_.max_velocity_ = opt_.min_velocity_;
    }
    if(opt_.collision_box_max_length_ < opt_.collision_box_min_length_) {
        ROS_ERROR("min length larger than max length!");
        opt_.collision_box_min_length_ = opt_.collision_box_max_length_;
    }
    if(opt_.collision_box_min_length_ < opt_.collision_box_crit_length_) {
        ROS_ERROR("min length smaller than crit length!");
        opt_.collision_box_crit_length_ = opt_.collision_box_min_length_;
    }
}

void PathFollower::setGoal(const FollowPathGoal &goal)
{
    pending_error_ = -1;

    if ( goal.path.poses.size() < 2 ) {
        ROS_ERROR( "Got an invalid path with less than two poses." );
        stop();
        pending_error_ = FollowPathResult::MOTION_STATUS_INTERNAL_ERROR;
        return;
    }

    setPath(goal.path);

    ROS_INFO_STREAM("Following path with " << goal.path.poses.size() << " poses.");
}

void PathFollower::setPath(const nav_msgs::Path& path)
{
    path_ = path;

    paths_.clear();

    // find segments
    findSegments(getController()->isOmnidirectional());

    controller_->reset();
}

void PathFollower::findSegments(bool only_one_segment)
{
    unsigned n = path_.poses.size();
    if(n < 2) {
        return;
    }

    Path current_segment;

    Waypoint last_point(path_.poses[0]);
    current_segment.push_back(last_point);

    int id = 0;

    for(unsigned i = 1; i < n; ++i){
        const Waypoint current_point(path_.poses[i]);

        // append to current segment
        current_segment.push_back(current_point);

        bool is_the_last_node = i == n-1;
        bool segment_ends_with_this_node = false;

        if(is_the_last_node) {
            // this is the last node
            segment_ends_with_this_node = true;

        } else {
            const Waypoint next_point(path_.poses[i+1]);

            // if angle between last direction and next direction to large -> segment ends
            double diff_last_x = current_point.x - last_point.x;
            double diff_last_y = current_point.y - last_point.y;
            double last_angle = std::atan2(diff_last_y, diff_last_x);

            double diff_next_x = next_point.x - current_point.x;
            double diff_next_y = next_point.y - current_point.y;
            double next_angle = std::atan2(diff_next_y, diff_next_x);

            double angle = MathHelper::AngleClamp(last_angle - next_angle);

            bool split_segment = std::abs(angle) > M_PI / 3.0;
            if(!only_one_segment && split_segment) {
                // new segment!
                // current node is the last one of the old segment
                segment_ends_with_this_node = true;
            }
        }

        visualizer_->drawArrow(id++, current_point, "paths", 0, 0, 0);
        if(segment_ends_with_this_node) {
            // Marker for subpaths
            visualizer_->drawMark(id++, ((geometry_msgs::Pose)current_point).position, "paths", 0.2,0.2,0.2);


            paths_.push_back(current_segment);
            current_segment.clear();

            if(!is_the_last_node) {
                // begin new segment
                // current node is also the first one of the new segment
                current_segment.push_back(current_point);
            }
        }

        last_point = current_point;
    }
}

void PathFollower::clearActive()
{
    if(active_behaviour_ != NULL) {
        delete active_behaviour_;
    }
    active_behaviour_ = NULL;
}


void PathFollower::beep(const std::vector<int> &beeps)
{
    ros::Time now = ros::Time::now();
    if(last_beep_ + beep_pause_ > now) {
        return;
    }

    last_beep_ = now;

    std_msgs::Int32MultiArray msg;

    msg.data.insert(msg.data.begin(), beeps.begin(), beeps.end());

    beeper_.publish(msg);
}
