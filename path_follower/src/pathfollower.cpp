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
#include <path_follower/obstacle_avoidance/obstacleavoider.h>

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
    controller_(nullptr),
    local_planner_(nullptr),
    obstacle_avoider_(nullptr),
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

    // Choose robot controller
    controller_factory_->construct(controller_, local_planner_, obstacle_avoider_);

    ROS_ASSERT_MSG(controller_ != nullptr, "Controller was not set");
    ROS_ASSERT_MSG(local_planner_ != nullptr, "Local Planner was not set");
    ROS_ASSERT_MSG(obstacle_avoider_ != nullptr, "Obstacle Avoider was not set");

    /*** Initialize supervisors ***/

    // register callback for new waypoint event.
    path_->registerNextWaypointCallback([this]() { supervisors_->notifyNewWaypoint(); });
    path_->registerNextWaypointCallback([this]() { local_planner_->reset(); });

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

    obstacle_avoider_->setObstacles(msg);
}



boost::variant<FollowPathFeedback, FollowPathResult> PathFollower::update()
{
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
        controller_->setCurrentPose(pose_tracker_->getRobotPose());
    }

    // Ask supervisor whether path following can continue
    Supervisor::State state(pose_tracker_->getRobotPose(),
                            getPath(),
                            obstacle_cloud_,
                            feedback);

    Supervisor::Result s_res = supervisors_->supervise(state);
    if (s_res.can_continue) {

        if(local_planner_->isNull()) {
            is_running_ = execute(feedback, result);
        } else  {
            //Begin Constraints and Scorers Construction
            //In case that more constraints or scorers need to be added, please add them
            //before Dis2Obst_Constraint/Scorer
            std::vector<Constraint::Ptr> constraints(2);
            std::vector<bool> fconstraints(2);
            if(opt_.c1()){
                constraints.at(0) = Dis2Path_Constraint::Ptr(new Dis2Path_Constraint);
            }
            fconstraints.at(0) = opt_.c1();

            if(opt_.c2()){
                constraints.at(1) = Dis2Obst_Constraint::Ptr(new Dis2Obst_Constraint);
            }
            fconstraints.at(1) = opt_.c2();

            std::vector<Scorer::Ptr> scorer(6);
            std::vector<double> wscorer(6);
            if(opt_.s1() != 0.0){
                scorer.at(0) = Dis2PathP_Scorer::Ptr(new Dis2PathP_Scorer);
            }
            wscorer.at(0) = opt_.s1();

            if(opt_.s2() != 0.0){
                scorer.at(1) = Dis2PathD_Scorer::Ptr(new Dis2PathD_Scorer);
            }
            wscorer.at(1) = opt_.s2();

            if(opt_.s3() != 0.0){
                scorer.at(2) = Curvature_Scorer::Ptr(new Curvature_Scorer);
            }
            wscorer.at(2) = opt_.s3();

            if(opt_.s4() != 0.0){
                scorer.at(3) = CurvatureD_Scorer::Ptr(new CurvatureD_Scorer);
            }
            wscorer.at(3) = opt_.s4();

            if(opt_.s5() != 0.0){
                scorer.at(4) = Level_Scorer::Ptr(new Level_Scorer);
            }
            wscorer.at(4) = opt_.s5();

            if(opt_.s6() != 0.0){
                scorer.at(5) = Dis2Obst_Scorer::Ptr(new Dis2Obst_Scorer);
            }
            wscorer.at(5) = opt_.s6();

            //End Constraints and Scorers Construction
            publishPathMarker();
            if(obstacle_cloud_ != nullptr){
                local_planner_->setObstacleCloud(obstacle_cloud_);
            }
            if(opt_.use_v()){
                local_planner_->setVelocity(pose_tracker_->getVelocity().linear);
            }

            bool path_search_failure = false;
            Path::Ptr local_path_whole(new Path("odom"));
            try {
                Path::Ptr local_path = local_planner_->updateLocalPath(constraints, scorer, fconstraints, wscorer, local_path_whole);
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
                ROS_ERROR_STREAM("no local path found. there is an obstacle.");
                feedback.status = path_msgs::FollowPathFeedback::MOTION_STATUS_OBSTACLE;
                controller_->stopMotion();

                // avoid RViz bug with empty paths!
                nav_msgs::Path path;
                path.header.stamp = ros::Time::now();
                path.header.frame_id = getFixedFrameId();
                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
                path.poses.push_back(pose);
                local_path_pub_.publish(path);

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

    } else {
        ROS_WARN("My supervisor told me to stop.");
        stop(s_res.status);
    }

    if(is_running_) {
        return feedback;
    } else {
        return result;
    }
}

void PathFollower::setStatus(int status)
{
    // TODO: don't use status this way...
}

PoseTracker& PathFollower::getPoseTracker()
{
    return *pose_tracker_;
}

RobotController *PathFollower::getController()
{
    return controller_.get();
}

ObstacleAvoider& PathFollower::getObstacleAvoider()
{
    return *obstacle_avoider_;
}

CoursePredictor &PathFollower::getCoursePredictor()
{
    return *course_predictor_;
}

Path::Ptr PathFollower::getPath()
{
    return path_;
}

std::string PathFollower::getFixedFrameId()
{
    return pose_tracker_->getFixedFrameId();
}

const PathFollowerParameters& PathFollower::getOptions() const
{
    return opt_;
}

Visualizer& PathFollower::getVisualizer() const
{
    return *visualizer_;
}

ros::NodeHandle& PathFollower::getNodeHandle()
{
    return node_handle_;
}

std::shared_ptr<ObstacleCloud const> PathFollower::getObstacleCloud() const
{
    return obstacle_cloud_;
}

bool PathFollower::isRunning() const
{
    return is_running_;
}

void PathFollower::start()
{
    //path_idx_.reset();

    course_predictor_->reset();
    controller_->reset();

    controller_->start();

    local_planner_->setGlobalPath(path_);
    local_planner_->setVelocity(vel_);

    g_robot_path_marker_.header.stamp = ros::Time();
    g_robot_path_marker_.points.clear();

    is_running_ = true;
}

void PathFollower::stop(int status)
{
    is_running_ = false;

    controller_->reset();
    controller_->stopMotion();

    pending_error_ = status;
}

void PathFollower::emergencyStop()
{
    stop(FollowPathResult::RESULT_STATUS_INTERNAL_ERROR);
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
        stop(-1);

        ROS_WARN("pending error");
        return DONE;
    }

    if(path_->empty()) {
        controller_->reset();
        result.status = FollowPathResult::RESULT_STATUS_SUCCESS;
        ROS_WARN("no path");
        return DONE;
    }

    visualizer_->drawArrow(getFixedFrameId(), 0, pose_tracker_->getRobotPoseMsg(), "slam pose", 2.0, 0.7, 1.0);

    RobotController::ControlStatus status = controller_->execute();

    switch(status)
    {
    case RobotController::ControlStatus::REACHED_GOAL:
        if(!local_planner_->isNull()) {
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
        ROS_INFO_STREAM("aborting, status=" << static_cast<int>(status));
        result.status = FollowPathResult::RESULT_STATUS_INTERNAL_ERROR;
        return DONE;
    }
}

void PathFollower::setGoal(const FollowPathGoal &goal)
{
    vel_ = goal.velocity;
    controller_->setVelocity(vel_);

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
}

void PathFollower::setPath(const path_msgs::PathSequence& path)
{
    path_->clear();

    // find segments
    findSegments(path, getController()->isOmnidirectional());

    controller_->reset();
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

    std::cout << "split the path into " << subpaths.size() << " sub paths" << std::endl;
    for(const SubPath& sp : subpaths) {
        std::cout << " - " << (sp.forward ? "forward" : "backward" ) << std::endl;
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
