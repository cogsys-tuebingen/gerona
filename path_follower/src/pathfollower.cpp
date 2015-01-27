#include <path_follower/pathfollower.h>

/// SYSTEM
#include <boost/foreach.hpp>
#include <Eigen/Core>
#include <utils_general/MathHelper.h>
#include <cmath>
#include <boost/assign.hpp>

/// ROS
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32MultiArray.h>

/// PROJECT
#include <std_msgs/Int32MultiArray.h>
// Controller/Models
#include <path_follower/legacy/robotcontroller_ackermann_pid.h>
#include <path_follower/legacy/robotcontroller_ackermann_orthexp.h>
#include <path_follower/legacy/robotcontroller_omnidrive_vv.h>
#include <path_follower/legacy/robotcontroller_omnidrive_orthexp.h>
// Supervisors
#include <path_follower/supervisor/pathlookout.h>
#include <path_follower/supervisor/waypointtimeout.h>
#include <path_follower/supervisor/distancetopathsupervisor.h>

using namespace path_msgs;
using namespace std;
using namespace Eigen;


namespace beep {
static std::vector<int> OBSTACLE_IN_PATH = boost::assign::list_of(25)(25)(25);
}

PathFollower::PathFollower(ros::NodeHandle &nh):
    controller_(NULL),
    course_predictor_(this),
    node_handle_(nh),
    follow_path_server_(nh, "follow_path", false),
    path_(new Path),
    pending_error_(-1),
    last_beep_(ros::Time::now()),
    beep_pause_(2.0),
    is_running_(false)
{
    // Init. action server
    follow_path_server_.registerGoalCallback(boost::bind(&PathFollower::followPathGoalCB, this));
    follow_path_server_.registerPreemptCallback(boost::bind(&PathFollower::followPathPreemptCB,this));

    cmd_pub_    = node_handle_.advertise<geometry_msgs::Twist> ("/cmd_vel", 10);
    speech_pub_ = node_handle_.advertise<std_msgs::String>("/speech", 0);
    beep_pub_   = node_handle_.advertise<std_msgs::Int32MultiArray>("/cmd_beep", 100);

    odom_sub_ = node_handle_.subscribe<nav_msgs::Odometry>("/odom", 1, &PathFollower::odometryCB, this);

    VectorFieldHistogram* vfh_ptr = opt_.use_vfh() ? &vfh_ : 0;

    // Choose robot controller
    ROS_INFO("Use robot controller '%s'", opt_.controller().c_str());
    if (opt_.controller() == "ackermann_pid") {
        controller_ = new RobotController_Ackermann_Pid(cmd_pub_, this, vfh_ptr);
    } else if (opt_.controller() == "omnidrive_vv") {
        controller_ = new RobotController_Omnidrive_VirtualVehicle(cmd_pub_, this);
    } else if (opt_.controller() == "omnidrive_orthexp") {
        controller_ = new RobotController_Omnidrive_OrthogonalExponential(cmd_pub_, this);
    } else if (opt_.controller() == "ackermann_orthexp") {
        controller_ = new RobotController_Ackermann_OrthogonalExponential(cmd_pub_, this);
    } else {
        ROS_FATAL("Unknown robot controller. Shutdown.");
        exit(1);
    }


    if(opt_.use_obstacle_map()) {
        obstacle_map_sub_ = node_handle_.subscribe<nav_msgs::OccupancyGrid>("/obstacle_map", 0, boost::bind(&PathFollower::obstacleMapCB, this, _1));
    } else {
        laser_front_sub_ = node_handle_.subscribe<sensor_msgs::LaserScan>("/scan/filtered", 10, boost::bind(&PathFollower::laserCB, this, _1, false));
        laser_back_sub_  = node_handle_.subscribe<sensor_msgs::LaserScan>("/scan/back/filtered", 10, boost::bind(&PathFollower::laserCB, this, _1, true));
    }
    controller_->getObstacleDetector()->setUseMap(opt_.use_obstacle_map());
    controller_->getObstacleDetector()->setUseScan(!opt_.use_obstacle_map());

    visualizer_ = Visualizer::getInstance();


    /*** Initialize supervisors ***/

    // register callback for new waypoint event.
    path_->registerNextWaypointCallback(boost::bind(&SupervisorChain::notifyNewWaypoint, &supervisors_));

    if (opt_.use_path_lookout()) {
        Supervisor::Ptr tmp(new PathLookout( opt_.use_obstacle_map() ));
        supervisors_.addSupervisor(tmp);
    }

    // Waypoint timeout
    double wpto;
    ros::param::param<double>("~waypoint_timeout", wpto, 10.0); //TODO: wrap all these param accesses with Parameters class
    Supervisor::Ptr waypoint_timeout(new WaypointTimeout(ros::Duration(wpto)));
    supervisors_.addSupervisor(waypoint_timeout);

    // Distance to path
    supervisors_.addSupervisor(Supervisor::Ptr(new DistanceToPathSupervisor(opt_.max_distance_to_path())));


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

    supervisors_.notifyNewGoal();
}

void PathFollower::followPathPreemptCB()
{
    controller_->stopMotion();
    pending_error_ = FollowPathResult::MOTION_STATUS_SUCCESS;
}

void PathFollower::odometryCB(const nav_msgs::OdometryConstPtr &odom)
{
    odometry_ = *odom;
}

void PathFollower::laserCB(const sensor_msgs::LaserScanConstPtr &scan, bool isBack)
{
    controller_->getObstacleDetector()->setScan(scan, isBack);
}

void PathFollower::obstacleMapCB(const nav_msgs::OccupancyGridConstPtr &map)
{
    controller_->getObstacleDetector()->setMap(map);

    if(opt_.use_vfh()) {
        vfh_.setMap(*map);
    }
}

bool PathFollower::updateRobotPose()
{
    if (getWorldPose(&robot_pose_, &robot_pose_msg_)) {
        course_predictor_.update();
        return true;
    } else {
        return false;
    }
}

bool PathFollower::getWorldPose(Vector3d *pose_vec , geometry_msgs::Pose *pose_msg) const
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
        global.header.frame_id=opt_.world_frame();
        pose_listener_.transformPose(opt_.robot_frame(),ros::Time(0),global,opt_.world_frame(),local);
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
        local.header.frame_id=opt_.robot_frame();
        pose_listener_.transformPose(opt_.world_frame(),ros::Time(0),local,opt_.robot_frame(),global);
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

        if(!is_running_) {
            start();
        }

        if (!updateRobotPose()) {
            ROS_ERROR("do not known own pose");
            is_running_ = false;
            result.status = FollowPathResult::MOTION_STATUS_SLAM_FAIL;
        }

        // Ask supervisor whether path following can continue
        Supervisor::State state(robot_pose_,
                                getPath(),
                                feedback);

        Supervisor::Result s_res = supervisors_.supervise(state);
        if (s_res.can_continue) {
            is_running_ = execute(feedback, result);
        } else {
            ROS_ERROR("My supervisor told me to stop.");
            is_running_ = false;
            result.status = s_res.status;
            controller_->stopMotion();
        }


        if (is_running_) {
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

void PathFollower::setStatus(int status)
{
    // TODO: don't use status this way...
}

bool PathFollower::isObstacleAhead(double course)
{
//    return false; //FIXME: Remove!!

    //! Factor which defines, how much the box is enlarged in curves.
    const float enlarge_factor = 0.5; // should this be a parameter?

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

    const float diff_to_min_velocity = v - opt_.min_velocity();

    const float norm = opt_.collision_box_velocity_saturation() - opt_.min_velocity();
    const float span = opt_.collision_box_max_length() - opt_.collision_box_min_length();
    const float interp = std::max(0.0f, diff_to_min_velocity) / std::max(norm, 0.001f);
    const float f = std::min(1.0f, opt_.collision_box_velocity_factor() * interp);

    float box_length = opt_.collision_box_min_length() + span * f;

    //ROS_DEBUG("Collision Box: v = %g -> len = %g", v, box_length);

    double distance_to_goal = path_->getCurrentSubPath().back().distanceTo(path_->getCurrentWaypoint());

    if(box_length > distance_to_goal) {
        box_length = distance_to_goal + 0.2;
    }

    if(box_length < opt_.collision_box_crit_length()) {
        box_length = opt_.collision_box_crit_length();
    }


    // call the obstacle detector. it is dependent of the controller als different driving models may require different
    // handling
    bool collision = controller_->getObstacleDetector()->isObstacleAhead(opt_.collision_box_width(), box_length, course,
                                                                         enlarge_factor);

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

CoursePredictor &PathFollower::getCoursePredictor()
{
    return course_predictor_;
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

Path::Ptr PathFollower::getPath()
{
    return path_;
}

void PathFollower::start()
{
    //path_idx_.reset();

    controller_->reset();

    controller_->start();
    controller_->setPath(getPath());

    is_running_ = true;
}

void PathFollower::stop()
{
    controller_->reset();

    controller_->stopMotion();
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

        ROS_WARN("pending error");
        return DONE;
    }

    if(path_->empty()) {
        controller_->reset();
        result.status = FollowPathResult::MOTION_STATUS_SUCCESS;
        ROS_WARN("no path");
        return DONE;
    }

    visualizer_->drawArrow(0, getRobotPoseMsg(), "slam pose", 2.0, 0.7, 1.0);

    RobotController::ControlStatus status = controller_->execute();

    controller_->publishCommand();

    switch(status)
    {
    case RobotController::SUCCESS:
        result.status = FollowPathResult::MOTION_STATUS_SUCCESS;
        return DONE;

    case RobotController::OBSTACLE:
        if (opt_.abort_if_obstacle_ahead()) {
            result.status = FollowPathResult::MOTION_STATUS_OBSTACLE;
            return DONE;
        } else {
            feedback.status = FollowPathFeedback::MOTION_STATUS_OBSTACLE;
            return MOVING;
        }

    case RobotController::MOVING:
        feedback.status = FollowPathFeedback::MOTION_STATUS_MOVING;
        return MOVING;

    default:
        ROS_INFO_STREAM("aborting, status=" << status);
        result.status = FollowPathResult::MOTION_STATUS_INTERNAL_ERROR;
        return DONE;
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
    path_->clear();

    // find segments
    findSegments(path, getController()->isOmnidirectional());

    controller_->reset();
}

void PathFollower::findSegments(const nav_msgs::Path& path, bool only_one_segment)
{
    unsigned n = path.poses.size();
    if(n < 2) {
        return;
    }

    vector<SubPath> subpaths;
    SubPath current_segment;

    Waypoint last_point(path.poses[0]);
    current_segment.push_back(last_point);

    int id = 0;

    for(unsigned i = 1; i < n; ++i){
        const Waypoint current_point(path.poses[i]);

        // append to current segment
        current_segment.push_back(current_point);

        bool is_the_last_node = i == n-1;
        bool segment_ends_with_this_node = false;

        if(is_the_last_node) {
            // this is the last node
            segment_ends_with_this_node = true;

        } else {
            const Waypoint next_point(path.poses[i+1]);

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


            subpaths.push_back(current_segment);
            current_segment.clear();

            if(!is_the_last_node) {
                // begin new segment
                // current node is also the first one of the new segment
                current_segment.push_back(current_point);
            }
        }

        last_point = current_point;
    }

    path_->setPath(subpaths);
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

    beep_pub_.publish(msg);
}
