#include <path_follower/pathfollower.h>

/// SYSTEM
#include <Eigen/Core>
#include <cslibs_utils/MathHelper.h>
#include <cmath>
#include <boost/assign.hpp>

/// ROS
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32MultiArray.h>
#include <tf_conversions/tf_eigen.h>

/// PROJECT
#include <path_follower/local_planner/local_planner.h>
#include <path_follower/controller/robotcontroller.h>
// Supervisors
#include <path_follower/supervisor/pathlookout.h>
#include <path_follower/supervisor/waypointtimeout.h>
#include <path_follower/supervisor/distancetopathsupervisor.h>
// Utils
#include <path_follower/utils/path_exceptions.h>
#include <path_follower/factory/controller_factory.h>
#include <path_follower/utils/coursepredictor.h>
#include <path_follower/utils/visualizer.h>
#include <path_follower/supervisor/supervisorchain.h>
#include <path_follower/utils/pose_tracker.h>
#include <path_follower/collision_avoidance/collision_avoider.h>

using namespace path_msgs;
using namespace std;
using namespace Eigen;

// toelrances which define when two waypoints are assumed as identical
const double WAYPOINT_POS_DIFF_TOL = 0.001;
const double WAYPOINT_ANGLE_DIFF_TOL = 0.01*M_PI/180.0;

PathFollower::PathFollower(ros::NodeHandle &nh):
    node_handle_(nh),
    pose_tracker_(new PoseTracker(opt_, nh)),
    controller_factory_(new ControllerFactory(*this)),
    supervisors_(new SupervisorChain()),
    course_predictor_(new CoursePredictor(pose_tracker_.get())),
    visualizer_(Visualizer::getInstance()),
    path_(new Path("map")),
    pending_error_(-1),
    is_running_(false),
    vel_(0.0)
{


    local_path_pub_ = node_handle_.advertise<nav_msgs::Path>("local_path", 1, true);
    whole_local_path_pub_ = node_handle_.advertise<nav_msgs::Path>("whole_local_path", 1, true);
    g_points_pub_ = node_handle_.advertise<visualization_msgs::Marker>("g_path_points", 10);

    /*** Initialize supervisors ***/

    // register callback for new waypoint event.
    path_->registerNextWaypointCallback([this]() { supervisors_->notifyNewWaypoint(); });

    if (opt_.supervisor_use_path_lookout()) {
        supervisors_->addSupervisor( Supervisor::Ptr(new PathLookout(*pose_tracker_)) );
    }

    // Waypoint timeout
    if (opt_.supervisor_use_waypoint_timeout()) {
        Supervisor::Ptr waypoint_timeout(
                    new WaypointTimeout(ros::Duration( opt_.supervisor_waypoint_timeout_time())));
        supervisors_->addSupervisor(waypoint_timeout);
    }

    // Distance to path
    if (opt_.supervisor_use_distance_to_path()) {
        supervisors_->addSupervisor(Supervisor::Ptr(
                                        new DistanceToPathSupervisor(opt_.supervisor_distance_to_path_max_dist())));
    }


    // path marker
    g_robot_path_marker_.header.frame_id = "odom";
    g_robot_path_marker_.header.stamp = ros::Time();
    g_robot_path_marker_.ns = "global robot path";
    g_robot_path_marker_.id = 75;
    g_robot_path_marker_.type = visualization_msgs::Marker::LINE_STRIP;
    g_robot_path_marker_.action = visualization_msgs::Marker::ADD;
    g_robot_path_marker_.pose.position.x = 0;
    g_robot_path_marker_.pose.position.y = 0;
    g_robot_path_marker_.pose.position.z = 0;
    g_robot_path_marker_.pose.orientation.x = 0.0;
    g_robot_path_marker_.pose.orientation.y = 0.0;
    g_robot_path_marker_.pose.orientation.z = 0.0;
    g_robot_path_marker_.pose.orientation.w = 1.0;
    g_robot_path_marker_.scale.x = 0.01;
    g_robot_path_marker_.scale.y = 0.0;
    g_robot_path_marker_.scale.z = 0.0;
    g_robot_path_marker_.color.a = 1.0;
    g_robot_path_marker_.color.r = 1.0;
    g_robot_path_marker_.color.g = 1.0;
    g_robot_path_marker_.color.b = 1.0;

    ROS_INFO("Initialisation done.");

}



PathFollower::~PathFollower()
{
}

void PathFollower::setObstacles(const std::shared_ptr<ObstacleCloud const> &msg)
{
    obstacle_cloud_ = msg;

    if(config_) {
        config_->collision_avoider_->setObstacles(msg);
    }
}


boost::variant<FollowPathFeedback, FollowPathResult> PathFollower::update()
{
    ROS_ASSERT(config_);

    FollowPathFeedback feedback;
    FollowPathResult result;

    if(!is_running_) {
        start();
    }

    if (!pose_tracker_->updateRobotPose()) {
        ROS_ERROR("do not known own pose");
        stop(FollowPathResult::RESULT_STATUS_SLAM_FAIL);

    } else {
        course_predictor_->update();
        config_->controller_->setCurrentPose(pose_tracker_->getRobotPose());
    }

    // Ask supervisor whether path following can continue
    Supervisor::State state(pose_tracker_->getRobotPose(),
                            path_,
                            obstacle_cloud_,
                            feedback);

    Supervisor::Result s_res = supervisors_->supervise(state);
    if(!s_res.can_continue) {
        ROS_WARN("My supervisor told me to stop.");
        stop(s_res.status);

        return result;

    }

    if(config_->local_planner_->isNull()) {
        is_running_ = execute(feedback, result);

    } else  {
        //End Constraints and Scorers Construction
        publishPathMarker();
        if(obstacle_cloud_ != nullptr){
            config_->local_planner_->setObstacleCloud(obstacle_cloud_);
        }
        if(opt_.local_planner.use_v()){
            config_->local_planner_->setVelocity(pose_tracker_->getVelocity().linear);
        }

        bool path_search_failure = false;
        Path::Ptr local_path_whole(new Path("odom"));
        try {
            Path::Ptr local_path = config_->local_planner_->updateLocalPath(local_path_whole);
            path_search_failure = local_path && local_path->empty();
            if(local_path && !path_search_failure) {
                nav_msgs::Path path;
                path.header.stamp = ros::Time::now();
                path.header.frame_id = getFixedFrameId();
                for(int i = 0, sub = local_path->subPathCount(); i < sub; ++i) {
                    const SubPath& p = local_path->getSubPath(i);
                    for(const Waypoint& wp : p.wps) {
                        geometry_msgs::PoseStamped pose;
                        pose.pose.position.x = wp.x;
                        pose.pose.position.y = wp.y;
                        pose.pose.orientation = tf::createQuaternionMsgFromYaw(wp.orientation);
                        path.poses.push_back(pose);
                    }
                }
                local_path_pub_.publish(path);
            }

            is_running_ = execute(feedback, result);

        } catch(const std::runtime_error& e) {
            ROS_ERROR_STREAM("Cannot find local_path: " << e.what());
            path_search_failure = true;
        }

        if(path_search_failure) {
            ROS_ERROR_STREAM_THROTTLE(1, "no local path found.");
            feedback.status = path_msgs::FollowPathFeedback::MOTION_STATUS_NO_LOCAL_PATH;
            config_->controller_->stopMotion();

            // avoid RViz bug with empty paths!
            nav_msgs::Path path;
            path.header.stamp = ros::Time::now();
            path.header.frame_id = getFixedFrameId();
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
            path.poses.push_back(pose);
            local_path_pub_.publish(path);

            return feedback;

        } else {
            if(local_path_whole->subPathCount() > 0){
                nav_msgs::Path wpath;
                wpath.header.stamp = ros::Time::now();
                wpath.header.frame_id = local_path_whole->getFrameId();
                for(int i = 0, sub = local_path_whole->subPathCount(); i < sub; ++i) {
                    const SubPath& p = local_path_whole->getSubPath(i);
                    for(const Waypoint& wp : p.wps) {
                        geometry_msgs::PoseStamped pose;
                        pose.pose.position.x = wp.x;
                        pose.pose.position.y = wp.y;
                        pose.pose.orientation = tf::createQuaternionMsgFromYaw(wp.orientation);
                        wpath.poses.push_back(pose);
                    }
                }
                whole_local_path_pub_.publish(wpath);
            }

            is_running_ = execute(feedback, result);
        }
    }

    if(is_running_) {
        return feedback;
    } else {
        return result;
    }
}

PoseTracker& PathFollower::getPoseTracker()
{
    return *pose_tracker_;
}

std::string PathFollower::getFixedFrameId() const
{
    return pose_tracker_->getFixedFrameId();
}

const PathFollowerParameters& PathFollower::getOptions() const
{
    return opt_;
}

ros::NodeHandle& PathFollower::getNodeHandle()
{
    return node_handle_;
}

bool PathFollower::isRunning() const
{
    return is_running_;
}

void PathFollower::start()
{
    ROS_ASSERT(config_);
    //path_idx_.reset();

    course_predictor_->reset();
    config_->controller_->reset();

    config_->controller_->start();

    config_->local_planner_->setGlobalPath(path_);
    config_->local_planner_->setVelocity(vel_);

    g_robot_path_marker_.header.stamp = ros::Time();
    g_robot_path_marker_.points.clear();

    is_running_ = true;
}

void PathFollower::stop(int status)
{
    ROS_ASSERT(config_);

    is_running_ = false;

    config_->controller_->reset();
    config_->controller_->stopMotion();

    pending_error_ = status;
}

void PathFollower::emergencyStop()
{
    ROS_WARN("***EMERGENCY STOP***");
    stop(FollowPathResult::RESULT_STATUS_INTERNAL_ERROR);
}

bool PathFollower::execute(FollowPathFeedback& feedback, FollowPathResult& result)
{
    ROS_ASSERT(config_);
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
        stop(-1);

        ROS_WARN("pending error");
        return DONE;
    }

    if(path_->empty()) {
        config_->controller_->reset();
        result.status = FollowPathResult::RESULT_STATUS_SUCCESS;
        ROS_WARN("no path");
        return DONE;
    }

    visualizer_->drawArrow(getFixedFrameId(), 0, pose_tracker_->getRobotPoseMsg(), "slam pose", 2.0, 0.7, 1.0);

    RobotController::ControlStatus status = config_->controller_->execute();

    switch(status)
    {
    case RobotController::ControlStatus::REACHED_GOAL:
        if(!config_->local_planner_->isNull()) {
            result.status = FollowPathResult::RESULT_STATUS_SUCCESS;
            return DONE;
            //feedback.status = FollowPathFeedback::MOTION_STATUS_OBSTACLE;
            ////return MOVING;
        } else {
            result.status = FollowPathResult::RESULT_STATUS_SUCCESS;
            return DONE;
        }

    case RobotController::ControlStatus::OBSTACLE:
        if (opt_.abort_if_obstacle_ahead()) {
            result.status = FollowPathResult::RESULT_STATUS_OBSTACLE;
            return DONE;
        } else {
            feedback.status = FollowPathFeedback::MOTION_STATUS_OBSTACLE;
            return MOVING;
        }

    case RobotController::ControlStatus::OKAY:
        feedback.status = FollowPathFeedback::MOTION_STATUS_MOVING;
        return MOVING;

    default:
        //ROS_INFO_STREAM("aborting, status=" << static_cast<int>(status));
        result.status = FollowPathResult::RESULT_STATUS_INTERNAL_ERROR;
        return DONE;
    }
}

PathFollowerConfigName PathFollower::goalToConfig(const FollowPathGoal &goal) const
{
    PathFollowerConfigName config;

    config.controller = goal.robot_controller.data;
    config.local_planner = goal.local_planner.data;
    config.collision_avoider = goal.collision_avoider.data;

    if(config.controller.empty()) {
        config.controller = opt_.controller();
    }
    if(config.local_planner.empty()) {
        config.local_planner = opt_.local_planner.local_planner();
    }
    if(config.collision_avoider.empty()) {
        config.collision_avoider = opt_.collision_avoider();
    }
    ROS_ASSERT_MSG(!config.controller.empty(), "No controller specified");
    ROS_ASSERT_MSG(!config.local_planner.empty(), "No local planner specified");

    return config;
}

void PathFollower::setGoal(const FollowPathGoal &goal)
{    
    // Choose robot controller
    PathFollowerConfigName config_name = goalToConfig(goal);

    auto pos = config_cache_.find(config_name);
    if(pos != config_cache_.end()) {
        config_ = pos->second;
    } else {
        config_ = controller_factory_->construct(config_name);
        config_cache_[config_name] = config_;
    }

    ROS_ASSERT(config_);
    if(obstacle_cloud_) {
        config_->collision_avoider_->setObstacles(obstacle_cloud_);
    }

    vel_ = goal.velocity;
    config_->controller_->setVelocity(vel_);

    pending_error_ = -1;

    if (goal.path.paths.empty()) {
        ROS_ERROR("Got an invalid path.");
        stop(FollowPathResult::RESULT_STATUS_INTERNAL_ERROR);
        return;
    }

    if(is_running_) {
        if(goal.init_mode != FollowPathGoal::INIT_MODE_CONTINUE) {
            ROS_ERROR("got a new goal, stopping");
            stop(FollowPathResult::RESULT_STATUS_SUCCESS);
        }
        is_running_ = false;
    }

    setPath(goal.path);

    supervisors_->notifyNewGoal();

    ROS_INFO_STREAM("Following path with " << goal.path.paths.size() << " segments.");

    ROS_INFO_STREAM("using follower configuration:\n- controller: " << config_name.controller <<
                    "\n- avoider: " << typeid(*config_->collision_avoider_).name() <<
                    "\n- local planner: " << config_name.local_planner);
}

void PathFollower::setPath(const path_msgs::PathSequence& path)
{
    ROS_ASSERT(config_);

    path_->clear();

    // find segments
    findSegments(path, config_->controller_->isOmnidirectional());

    config_->controller_->reset();
}

namespace {
double angleDifference(double s1, double s2) {
    double a1 = std::atan2(std::sin(s1), std::cos(s1));
    double a2 = std::atan2(std::sin(s2), std::cos(s2));
    return MathHelper::AngleClamp(a1 - a2);
}
}

void PathFollower::findSegments(const path_msgs::PathSequence& path, bool only_one_segment)
{
    vector<SubPath> subpaths;

    for(const path_msgs::DirectionalPath& dp : path.paths) {
        SubPath sp(dp.forward);
        for(const geometry_msgs::PoseStamped& ps : dp.poses) {
            sp.wps.emplace_back(ps);
        }
        subpaths.push_back(sp);
    }

    path_->setPath(subpaths);
}

void PathFollower::publishPathMarker(){
    Eigen::Vector3d current_pose = pose_tracker_->getRobotPose();
    geometry_msgs::Point pt;
    pt.x = current_pose[0];
    pt.y = current_pose[1];
    g_robot_path_marker_.points.push_back(pt);

    g_points_pub_.publish(g_robot_path_marker_);
}
