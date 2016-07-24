#include <path_follower/pathfollower.h>

/// SYSTEM
#include <Eigen/Core>
#include <utils_general/MathHelper.h>
#include <cmath>
#include <boost/assign.hpp>

/// ROS
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32MultiArray.h>
#include <tf_conversions/tf_eigen.h>

/// PROJECT
// Controller/Models
#include <path_follower/controller/robotcontroller_ackermann_pid.h>
#include <path_follower/controller/robotcontrollertrailer.h>

#include <path_follower/legacy/robotcontroller_ackermann_orthexp.h>
#include <path_follower/legacy/robotcontroller_ackermann_purepursuit.h>
#include <path_follower/legacy/robotcontroller_ackermann_kinematic.h>
#include <path_follower/legacy/robotcontroller_ackermann_stanley.h>
#include <path_follower/legacy/robotcontroller_4ws_purepursuit.h>
#include <path_follower/legacy/robotcontroller_4ws_stanley.h>
#include <path_follower/legacy/robotcontroller_4ws_inputscaling.h>
#include <path_follower/legacy/robotcontroller_unicycle_inputscaling.h>
#include <path_follower/legacy/robotcontroller_omnidrive_orthexp.h>
#include <path_follower/legacy/robotcontroller_differential_orthexp.h>
#include <path_follower/legacy/robotcontroller_kinematic_SLP.h>
#include <path_follower/legacy/robotcontroller_dynamic_SLP.h>
#include <path_follower/legacy/robotcontroller_kinematic_SSG.h>
// Supervisors
#include <path_follower/supervisor/pathlookout.h>
#include <path_follower/supervisor/waypointtimeout.h>
#include <path_follower/supervisor/distancetopathsupervisor.h>
// Obstacle Avoiders
#include <path_follower/obstacle_avoidance/noneavoider.hpp>
#include <path_follower/obstacle_avoidance/obstacledetectorackermann.h>
#include <path_follower/obstacle_avoidance/obstacledetectoromnidrive.h>
#include <path_follower/obstacle_avoidance/obstacledetectorpatsy.h>
// Utils
#include <path_follower/utils/path_exceptions.h>

#include <path_follower/local_planner/local_planner_null.h>
#include <path_follower/local_planner/local_planner_transformer.h>
#include <path_follower/local_planner/local_planner_bfs.h>
#include <path_follower/local_planner/local_planner_astar.h>
#include <path_follower/local_planner/local_planner_astar_dynamic.h>

using namespace path_msgs;
using namespace std;
using namespace Eigen;

// toelrances which define when two waypoints are assumed as identical
const double WAYPOINT_POS_DIFF_TOL = 0.001;
const double WAYPOINT_ANGLE_DIFF_TOL = 0.01*M_PI/180.0;


namespace beep {
static std::vector<int> OBSTACLE_IN_PATH = boost::assign::list_of(25)(25)(25);
}

PathFollower::PathFollower(ros::NodeHandle &nh):
    node_handle_(nh),
    follow_path_server_(nh, "follow_path", false),
    controller_(nullptr),
    local_planner_(nullptr),
    obstacle_avoider_(nullptr),
    course_predictor_(this),
    path_(new Path("/map")),
    pending_error_(-1),
    last_beep_(ros::Time::now()),
    beep_pause_(2.0),
    is_running_(false)
{

    // Init. action server
    follow_path_server_.registerGoalCallback([this]() { followPathGoalCB(); });
    follow_path_server_.registerPreemptCallback([this]() {followPathPreemptCB(); });

    speech_pub_ = node_handle_.advertise<std_msgs::String>("/speech", 0);
    beep_pub_   = node_handle_.advertise<std_msgs::Int32MultiArray>("/cmd_beep", 100);
    local_path_pub_ = node_handle_.advertise<nav_msgs::Path>("local_path", 1, true);

    odom_sub_ = node_handle_.subscribe<nav_msgs::Odometry>("/odom", 1, &PathFollower::odometryCB, this);

    // Choose robot controller
    ROS_INFO("Use robot controller '%s'", opt_.controller().c_str());
    if (opt_.controller() == "ackermann_pid") {
        if (opt_.obstacle_avoider_use_collision_box())
            obstacle_avoider_ = std::make_shared<ObstacleDetectorAckermann>(&pose_listener_);
        controller_ = std::make_shared<RobotController_Ackermann_Pid>(this);

    } else if (opt_.controller() == "ackermann_purepursuit") {
        if (opt_.obstacle_avoider_use_collision_box())
            obstacle_avoider_ = std::make_shared<ObstacleDetectorAckermann>(&pose_listener_);
        controller_ = std::make_shared<Robotcontroller_Ackermann_PurePursuit>(this);

    } else if (opt_.controller() == "ackermann_kinematic") {
        if (opt_.obstacle_avoider_use_collision_box())
            obstacle_avoider_ = std::make_shared<ObstacleDetectorAckermann>(&pose_listener_);
        controller_ = std::make_shared<RobotController_Ackermann_Kinematic>(this);

    } else if (opt_.controller() == "ackermann_stanley") {
        if (opt_.obstacle_avoider_use_collision_box())
            obstacle_avoider_ = std::make_shared<ObstacleDetectorAckermann>(&pose_listener_);
        controller_ = std::make_shared<RobotController_Ackermann_Stanley>(this);

    } else if (opt_.controller() == "4ws_purepursuit") {
        if (opt_.obstacle_avoider_use_collision_box())
            obstacle_avoider_ = std::make_shared<ObstacleDetectorAckermann>(&pose_listener_);
        controller_ = std::make_shared<RobotController_4WS_PurePursuit>(this);

    } else if (opt_.controller() == "4ws_stanley") {
        if (opt_.obstacle_avoider_use_collision_box())
            obstacle_avoider_ = std::make_shared<ObstacleDetectorAckermann>(&pose_listener_);
        controller_ = std::make_shared<RobotController_4WS_Stanley>(this);

    } else if (opt_.controller() == "4ws_inputscaling") {
        if (opt_.obstacle_avoider_use_collision_box())
            obstacle_avoider_ = std::make_shared<ObstacleDetectorAckermann>(&pose_listener_);
        controller_ = std::make_shared<RobotController_4WS_InputScaling>(this);

    } else if (opt_.controller() == "unicycle_inputscaling") {
        if (opt_.obstacle_avoider_use_collision_box())
            obstacle_avoider_ = std::make_shared<ObstacleDetectorAckermann>(&pose_listener_);
        controller_ = std::make_shared<RobotController_Unicycle_InputScaling>(this);

    } else if (opt_.controller() == "patsy_pid") {
        std::shared_ptr<RobotControllerTrailer> ctrl(new RobotControllerTrailer(this,&this->node_handle_));
        if (opt_.obstacle_avoider_use_collision_box())
            obstacle_avoider_ = std::make_shared<ObstacleDetectorPatsy>(&pose_listener_,ctrl.get());
        controller_ = ctrl;

    } else if (opt_.controller() == "omnidrive_orthexp") {
        if (opt_.obstacle_avoider_use_collision_box())
            obstacle_avoider_ = std::make_shared<ObstacleDetectorOmnidrive>(&pose_listener_);
        controller_ = std::make_shared<RobotController_Omnidrive_OrthogonalExponential>(this);

    } else if (opt_.controller() == "ackermann_orthexp") {
        if (opt_.obstacle_avoider_use_collision_box())
            obstacle_avoider_ = std::make_shared<ObstacleDetectorOmnidrive>(&pose_listener_);
        controller_ = std::make_shared<RobotController_Ackermann_OrthogonalExponential>(this);

    } else if (opt_.controller() == "differential_orthexp") {
        if (opt_.obstacle_avoider_use_collision_box())
            obstacle_avoider_ = std::make_shared<ObstacleDetectorOmnidrive>(&pose_listener_);
        controller_ = std::make_shared<RobotController_Differential_OrthogonalExponential>(this);

    } else if (opt_.controller() == "kinematic_SLP") {
        if (opt_.obstacle_avoider_use_collision_box())
            obstacle_avoider_ = std::make_shared<ObstacleDetectorAckermann>(&pose_listener_);
        controller_ = std::make_shared<RobotController_Kinematic_SLP>(this);

    } else if (opt_.controller() == "dynamic_SLP") {
        if (opt_.obstacle_avoider_use_collision_box())
            obstacle_avoider_ = std::make_shared<ObstacleDetectorAckermann>(&pose_listener_);
        controller_ = std::make_shared<RobotController_Dynamic_SLP>(this);
    } else if (opt_.controller() == "kinematic_SSG") {
        if (opt_.obstacle_avoider_use_collision_box())
            obstacle_avoider_ = std::make_shared<ObstacleDetectorAckermann>(&pose_listener_);
        controller_ = std::make_shared<RobotController_Kinematic_SSG>(this);
    } else {
        ROS_FATAL("Unknown robot controller. Shutdown.");
        exit(1);
    }

    // Choose Local Planner Algorithm
    ROS_INFO("Use local planner algorithm '%s'", opt_.algo().c_str());
    if(opt_.algo() == "AStar"){
        local_planner_ = std::make_shared<LocalPlannerAStar>(*this, pose_listener_,ros::Duration(2.0));
    }else if(opt_.algo() == "AStarDynamic"){
        local_planner_ = std::make_shared<LocalPlannerAStarDynamic>(*this, pose_listener_,ros::Duration(0.5));
    }else if(opt_.algo() == "BFS"){
        local_planner_ = std::make_shared<LocalPlannerBFS>(*this, pose_listener_,ros::Duration(1.0));
    }else if(opt_.algo() == "Transformer"){
        local_planner_ = std::make_shared<LocalPlannerTransformer>(*this, pose_listener_,ros::Duration(1.0));
    }else if(opt_.algo() == "NULL"){
        local_planner_ = std::make_shared<LocalPlannerNull>(*this, pose_listener_);
    }else {
        ROS_FATAL("Unknown local planner algorithm. Shutdown.");
        exit(1);
    }

    ROS_INFO("Constraint usage [%s, %s]", opt_.c1() ? "true" : "false",
             opt_.c2() ? "true" : "false");
    ROS_INFO("Scorer usage [%s, %s, %s, %s, %s, %s]", (opt_.s1() != 0.0) ? "true" : "false",
             (opt_.s2() != 0.0) ? "true" : "false", (opt_.s3() != 0.0) ? "true" : "false",
             (opt_.s4() != 0.0) ? "true" : "false",(opt_.s5() != 0.0) ? "true" : "false",
             (opt_.s6() != 0.0) ? "true" : "false");

    obstacle_cloud_sub_ = node_handle_.subscribe<ObstacleCloud>("/obstacles", 10,
                                                                &PathFollower::obstacleCloudCB, this);

    visualizer_ = Visualizer::getInstance();


    /*** Initialize supervisors ***/

    // register callback for new waypoint event.
    path_->registerNextWaypointCallback([this]() { supervisors_.notifyNewWaypoint(); });

    if (opt_.supervisor_use_path_lookout()) {
        supervisors_.addSupervisor( Supervisor::Ptr(new PathLookout(&pose_listener_)) );
    }

    // Waypoint timeout
    if (opt_.supervisor_use_waypoint_timeout()) {
        Supervisor::Ptr waypoint_timeout(
                    new WaypointTimeout(ros::Duration( opt_.supervisor_waypoint_timeout_time())));
        supervisors_.addSupervisor(waypoint_timeout);
    }

    // Distance to path
    if (opt_.supervisor_use_distance_to_path()) {
        supervisors_.addSupervisor(Supervisor::Ptr(
                                       new DistanceToPathSupervisor(opt_.supervisor_distance_to_path_max_dist())));
    }


    //  if no obstacle avoider was set, use the none-avoider
    if (!obstacle_avoider_) {
        obstacle_avoider_ = std::make_shared<NoneAvoider>();
    }

    follow_path_server_.start();
    ROS_INFO("Initialisation done.");

}



PathFollower::~PathFollower()
{
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
    pending_error_ = FollowPathResult::RESULT_STATUS_SUCCESS;
}

void PathFollower::odometryCB(const nav_msgs::OdometryConstPtr &odom)
{
    odometry_ = *odom;

    robot_pose_odom_msg_ = odometry_.pose.pose;

    robot_pose_odom_.x() = robot_pose_odom_msg_.position.x;
    robot_pose_odom_.y() = robot_pose_odom_msg_.position.y;
    robot_pose_odom_.z() = tf::getYaw(robot_pose_odom_msg_.orientation);
}

void PathFollower::obstacleCloudCB(const ObstacleCloud::ConstPtr &msg)
{
    obstacle_cloud_ = msg;
}

bool PathFollower::updateRobotPose()
{
    if (getWorldPose(&robot_pose_world_, &robot_pose_world_msg_)) {
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

    if(pose_msg != nullptr) {
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
        global.header.frame_id = getFixedFrameId();
        pose_listener_.transformPose(opt_.robot_frame(),ros::Time(0),global, global.header.frame_id,local);
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
        pose_listener_.transformPose(getFixedFrameId(),ros::Time(0),local,opt_.robot_frame(),global);
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
        try {
            ros::spinOnce();
            update();
            rate.sleep();
        } catch (const EmergencyBreakException &e) {
            ROS_ERROR("Emergency Break [status %d]: %s", e.status_code, e.what());
            controller_->stopMotion();

            FollowPathResult result;
            result.status = e.status_code;
            follow_path_server_.setAborted(result);
        }
    }
}

void PathFollower::update()
{
    if (follow_path_server_.isActive()) {
        FollowPathFeedback feedback;
        FollowPathResult result;

        if(follow_path_server_.isPreemptRequested()) {
            if(is_running_) {
                is_running_ = false;
                stop();
                follow_path_server_.setPreempted();
                return;
            }
        }

        if(!is_running_) {
            start();
        }

        if (!updateRobotPose()) {
            ROS_ERROR("do not known own pose");
            is_running_ = false;
            result.status = FollowPathResult::RESULT_STATUS_SLAM_FAIL;
        }

        // Ask supervisor whether path following can continue
        Supervisor::State state(robot_pose_world_,
                                getPath(),
                                obstacle_cloud_,
                                feedback);

        Supervisor::Result s_res = supervisors_.supervise(state);
        if (s_res.can_continue) {
            //Begin Constraints and Scorers Construction
            if(local_planner_->isNull()) {
                is_running_ = execute(feedback, result);

            } else {
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
                    scorer.at(0) = Dis2Start_Scorer::Ptr(new Dis2Start_Scorer);
                }
                wscorer.at(0) = opt_.s1();

                if(opt_.s2() != 0.0){
                    scorer.at(1) = Dis2PathP_Scorer::Ptr(new Dis2PathP_Scorer);
                }
                wscorer.at(1) = opt_.s2();

                if(opt_.s3() != 0.0){
                    scorer.at(2) = Dis2PathI_Scorer::Ptr(new Dis2PathI_Scorer);
                }
                wscorer.at(2) = opt_.s3();

                if(opt_.s4() != 0.0){
                    scorer.at(3) = Dis2PathD_Scorer::Ptr(new Dis2PathD_Scorer);
                }
                wscorer.at(3) = opt_.s4();

                if(opt_.s5() != 0.0){
                    scorer.at(4) = Dis2Obst_Scorer::Ptr(new Dis2Obst_Scorer);
                }
                wscorer.at(4) = opt_.s5();

                if(opt_.s6() != 0.0){
                    scorer.at(5) = Curvature_Scorer::Ptr(new Curvature_Scorer);
                }
                wscorer.at(5) = opt_.s6();

                //End Constraints and Scorers Construction
                if(obstacle_cloud_ != nullptr){
                    local_planner_->setObstacleCloud(obstacle_cloud_);
                }
                local_planner_->setVelocity(getVelocity().linear);

                bool path_search_failure = false;
                try {
                    Path::Ptr local_path = local_planner_->updateLocalPath(constraints, scorer, fconstraints, wscorer);
                    path_search_failure = local_path && local_path->empty();
                    if(local_path && !path_search_failure) {
                        nav_msgs::Path path;
                        path.header.stamp = ros::Time::now();
                        path.header.frame_id = getFixedFrameId();
                        for(int i = 0, sub = local_path->subPathCount(); i < sub; ++i) {
                            const SubPath& p = local_path->getSubPath(i);
                            for(const Waypoint& wp : p) {
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
                    is_running_ = execute(feedback, result);
                }
            }

        } else {
            ROS_WARN("My supervisor told me to stop.");
            is_running_ = false;
            result.status = s_res.status;
            controller_->stopMotion();
        }


        if (is_running_) {
            follow_path_server_.publishFeedback(feedback);
        } else {
            if (result.status == FollowPathResult::RESULT_STATUS_SUCCESS) {
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

bool PathFollower::callObstacleAvoider(MoveCommand *cmd)
{
    if (obstacle_avoider_ == nullptr) {
        ROS_WARN_ONCE("No obstacle avoider selected. Obstacle avoidace is deactivated!");
        return false;
    }

    if (obstacle_cloud_ == nullptr) {
        ROS_ERROR_THROTTLE(1, "No obstacle cloud received. Obstacle avoidace is skipped!");
        return false;
    }

    ObstacleAvoider::State state(path_, opt_);

    return obstacle_avoider_->avoid(cmd, obstacle_cloud_, state);
}

RobotController *PathFollower::getController()
{
    return controller_.get();
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
    if(local_planner_->isNull()) {
        return robot_pose_world_;
    } else {
        return robot_pose_odom_;
    }
}

const geometry_msgs::Pose &PathFollower::getRobotPoseMsg() const
{
    if(local_planner_->isNull()) {
        return robot_pose_world_msg_;
    } else {
        return robot_pose_odom_msg_;
    }
}

Path::Ptr PathFollower::getPath()
{
    return path_;
}

std::string PathFollower::getFixedFrameId()
{
    if(local_planner_->isNull()) {
        return opt_.world_frame();
    } else {
        return opt_.odom_frame();
    }
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

void PathFollower::start()
{
    //path_idx_.reset();

    controller_->reset();

    controller_->start();

    local_planner_->setGlobalPath(path_);
    local_planner_->setVelocity(getVelocity().linear);

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
        result.status = FollowPathResult::RESULT_STATUS_SUCCESS;
        ROS_WARN("no path");
        return DONE;
    }

    visualizer_->drawArrow(getFixedFrameId(), 0, getRobotPoseMsg(), "slam pose", 2.0, 0.7, 1.0);

    RobotController::ControlStatus status = controller_->execute();

    switch(status)
    {
    case RobotController::ControlStatus::REACHED_GOAL:
        result.status = FollowPathResult::RESULT_STATUS_SUCCESS;
        return DONE;

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
    pending_error_ = -1;

    if ( goal.path.poses.size() < 2 ) {
        ROS_ERROR("Got an invalid path with less than two poses.");
        stop();
        pending_error_ = FollowPathResult::RESULT_STATUS_INTERNAL_ERROR;
        return;
    }

    if(is_running_) {
        ROS_ERROR("got a new goal, stopping");
        stop();
        is_running_ = false;
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

namespace {
double angleDifference(double s1, double s2) {
    double a1 = std::atan2(std::sin(s1), std::cos(s1));
    double a2 = std::atan2(std::sin(s2), std::cos(s2));
    return MathHelper::AngleClamp(a1 - a2);
}
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
    bool is_duplicate_wp ;

    // TODO: does this really do what we think it does?
    // FIXME: refactor this method
    for(unsigned i = 1; i < n; ++i){
        const Waypoint current_point(path.poses[i]);

        // filter identical points that lead to the "unsorted" alglib exception
        const bool is_identical = std::abs(current_point.x - last_point.x) < 1e-4
                && std::abs(current_point.y - last_point.y) < 1e-4;
        if(!is_identical) {
            // append to current segment if not in "filter" mode
            current_segment.push_back(current_point);
        }

        double diff_last_x = current_point.x - last_point.x;
        double diff_last_y = current_point.y - last_point.y;
        double diff_last_angle = MathHelper::AngleDelta(current_point.orientation, last_point.orientation);
        if (diff_last_x*diff_last_x+diff_last_y*diff_last_y<WAYPOINT_POS_DIFF_TOL*WAYPOINT_POS_DIFF_TOL
                && fabs(diff_last_angle)<WAYPOINT_ANGLE_DIFF_TOL) {
            // duplicate waypoint
            is_duplicate_wp = true;
        } else {
            is_duplicate_wp = false;
        }

        bool is_the_last_node = i == n-1;
        bool segment_ends_with_this_node = false;

        if(is_the_last_node) {
            // this is the last node
            segment_ends_with_this_node = true;

        } else if (!is_identical) {
            const Waypoint next_point(path.poses[i+1]);

            // if angle between last direction and next direction to large -> segment ends
            double diff_last_x = current_point.x - last_point.x;
            double diff_last_y = current_point.y - last_point.y;
            double last_angle = std::atan2(diff_last_y, diff_last_x);

            double diff_next_x = next_point.x - current_point.x;
            double diff_next_y = next_point.y - current_point.y;
            double next_angle = std::atan2(diff_next_y, diff_next_x);

            double angle = angleDifference(last_angle, next_angle);//MathHelper::AngleClamp(last_angle - next_angle);

            std::cerr << angle << std::endl;

            bool split_segment = std::abs(angle) > M_PI / 3.0;

            if(!only_one_segment && split_segment && !is_duplicate_wp) {
                // new segment!
                // current node is the last one of the old segment
                segment_ends_with_this_node = true;
            }
        }

        visualizer_->drawArrow(path.header.frame_id, id++, current_point, "paths", 0, 0, 0);
        if(segment_ends_with_this_node) {
            // Marker for subpaths
            visualizer_->drawMark(id++, ((geometry_msgs::Pose)current_point).position, "paths", 0.2,0.2,0.2);

            current_segment.pop_back();
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

    std::cout << "split the path into " << subpaths.size() << " sub paths" << std::endl;

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
