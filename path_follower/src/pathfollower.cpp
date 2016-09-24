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
#include <path_follower/controller/robotcontroller_ICR_CCW.h>

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
#include <path_follower/local_planner/local_planner_bfs_static.h>
#include <path_follower/local_planner/local_planner_bfs_reconf.h>
#include <path_follower/local_planner/local_planner_astar_n_static.h>
#include <path_follower/local_planner/local_planner_astar_g_static.h>
#include <path_follower/local_planner/local_planner_astar_n_reconf.h>
#include <path_follower/local_planner/local_planner_astar_g_reconf.h>
#include <path_follower/local_planner/local_planner_thetastar_n_static.h>
#include <path_follower/local_planner/local_planner_thetastar_g_static.h>
#include <path_follower/local_planner/local_planner_thetastar_n_reconf.h>
#include <path_follower/local_planner/local_planner_thetastar_g_reconf.h>

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
    is_running_(false),
    vel_(0.0)
{

    // Init. action server
    follow_path_server_.registerGoalCallback([this]() { followPathGoalCB(); });
    follow_path_server_.registerPreemptCallback([this]() {followPathPreemptCB(); });

    speech_pub_ = node_handle_.advertise<std_msgs::String>("/speech", 0);
    beep_pub_   = node_handle_.advertise<std_msgs::Int32MultiArray>("/cmd_beep", 100);
    local_path_pub_ = node_handle_.advertise<nav_msgs::Path>("local_path", 1, true);
    whole_local_path_pub_ = node_handle_.advertise<nav_msgs::Path>("whole_local_path", 1, true);
    g_points_pub_ = node_handle_.advertise<visualization_msgs::Marker>("g_path_points", 10);

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
    } else if (opt_.controller() == "ICR_CCW") {
        if (opt_.obstacle_avoider_use_collision_box())
            obstacle_avoider_ = std::make_shared<ObstacleDetectorAckermann>(&pose_listener_);
        controller_ = std::make_shared<RobotController_ICR_CCW>(this);
    } else {
        ROS_FATAL("Unknown robot controller. Shutdown.");
        exit(1);
    }

    // Choose Local Planner Algorithm
    ROS_INFO("Use local planner algorithm '%s'", opt_.algo().c_str());
    if(opt_.algo() == "AStar"){
        local_planner_ = std::make_shared<LocalPlannerAStarNStatic>(*this, pose_listener_,ros::Duration(opt_.uinterval()));
    }else if(opt_.algo() == "AStarG"){
        local_planner_ = std::make_shared<LocalPlannerAStarGStatic>(*this, pose_listener_,ros::Duration(opt_.uinterval()));
    }else if(opt_.algo() == "ThetaStar"){
        local_planner_ = std::make_shared<LocalPlannerThetaStarNStatic>(*this, pose_listener_,ros::Duration(opt_.uinterval()));
    }else if(opt_.algo() == "ThetaStarG"){
        local_planner_ = std::make_shared<LocalPlannerThetaStarGStatic>(*this, pose_listener_,ros::Duration(opt_.uinterval()));
    }else if(opt_.algo() == "AStarR"){
        local_planner_ = std::make_shared<LocalPlannerAStarNReconf>(*this, pose_listener_,ros::Duration(opt_.uinterval()));
    }else if(opt_.algo() == "AStarGR"){
        local_planner_ = std::make_shared<LocalPlannerAStarGReconf>(*this, pose_listener_,ros::Duration(opt_.uinterval()));
    }else if(opt_.algo() == "ThetaStarR"){
        local_planner_ = std::make_shared<LocalPlannerThetaStarNReconf>(*this, pose_listener_,ros::Duration(opt_.uinterval()));
    }else if(opt_.algo() == "ThetaStarGR"){
        local_planner_ = std::make_shared<LocalPlannerThetaStarGReconf>(*this, pose_listener_,ros::Duration(opt_.uinterval()));
    }else if(opt_.algo() == "BFS"){
        local_planner_ = std::make_shared<LocalPlannerBFSStatic>(*this, pose_listener_,ros::Duration(opt_.uinterval()));
    }else if(opt_.algo() == "BFSR"){
        local_planner_ = std::make_shared<LocalPlannerBFSReconf>(*this, pose_listener_,ros::Duration(opt_.uinterval()));
    }else if(opt_.algo() == "Transformer"){
        local_planner_ = std::make_shared<LocalPlannerTransformer>(*this, pose_listener_,ros::Duration(opt_.uinterval()));
    }else if(opt_.algo() == "NULL"){
        local_planner_ = std::make_shared<LocalPlannerNull>(*this, pose_listener_);
    }else {
        ROS_FATAL("Unknown local planner algorithm. Shutdown.");
        exit(1);
    }

    local_planner_->setParams(opt_.nnodes(), opt_.ic(), opt_.dis2p(), opt_.dis2o(),
                              opt_.s_angle(), opt_.ia(), opt_.lmf(),opt_.depth(),
                              opt_.mu(), opt_.ef());

    ROS_INFO("Maximum number of allowed nodes: %d", opt_.nnodes());
    ROS_INFO("Maximum tree depth: %d", opt_.depth());
    ROS_INFO("Update Interval: %.3f", opt_.uinterval());
    ROS_INFO("Maximal distance from path: %.3f", opt_.dis2p());
    ROS_INFO("Minimal distance to an obstacle: %.3f", opt_.dis2o());
    ROS_INFO("Steering angle: %.3f", opt_.s_angle());
    ROS_INFO("Intermediate Configurations: %d",opt_.ic());
    ROS_INFO("Intermediate Angles: %d",opt_.ia());
    ROS_INFO("Using current velocity: %s",opt_.use_v() ? "true" : "false");
    ROS_INFO("Length multiplying factor: %.3f",opt_.lmf());
    ROS_INFO("Coefficient of friction: %.3f",opt_.mu());
    ROS_INFO("Exponent factor: %.3f",opt_.ef());

    ROS_INFO("Constraint usage [%s, %s]", opt_.c1() ? "true" : "false",
             opt_.c2() ? "true" : "false");
    ROS_INFO("Scorer usage [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", opt_.s1(),opt_.s2(),
             opt_.s3(), opt_.s4(), opt_.s5(), opt_.s6());

    obstacle_cloud_sub_ = node_handle_.subscribe<ObstacleCloud>("/obstacles", 10,
                                                                &PathFollower::obstacleCloudCB, this);

    visualizer_ = Visualizer::getInstance();


    /*** Initialize supervisors ***/

    // register callback for new waypoint event.
    path_->registerNextWaypointCallback([this]() { supervisors_.notifyNewWaypoint(); });
    path_->registerNextWaypointCallback([this]() { local_planner_->reset(); });

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

    // path marker
    g_robot_path_marker_.header.frame_id = "/odom";
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

    follow_path_server_.start();
    ROS_INFO("Initialisation done.");

}



PathFollower::~PathFollower()
{
}


void PathFollower::followPathGoalCB()
{
    latest_goal_ = follow_path_server_.acceptNewGoal();
    ROS_INFO("Start Action!");

    // stop current goal
    if(latest_goal_->init_mode != FollowPathGoal::INIT_MODE_CONTINUE) {
        stop();
    }

    vel_ = latest_goal_->velocity;
    controller_->setVelocity(vel_);
    setGoal(*latest_goal_);

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
        controller_->setCurrentPose(robot_pose_world_);
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
//                if(latest_goal_->init_mode != FollowPathGoal::INIT_MODE_CONTINUE) {
                    stop();
//                }
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
                    local_planner_->setVelocity(getVelocity().linear);
                }

                bool path_search_failure = false;
                Path::Ptr local_path_whole(new Path("/odom"));
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

ObstacleCloud::ConstPtr PathFollower::getObstacleCloud() const
{
    return obstacle_cloud_;
}

void PathFollower::start()
{
    //path_idx_.reset();

    controller_->reset();

    controller_->start();

    local_planner_->setGlobalPath(path_);
    local_planner_->setVelocity(vel_);

    g_robot_path_marker_.header.stamp = ros::Time();
    g_robot_path_marker_.points.clear();

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
    pending_error_ = -1;

    if (goal.path.paths.empty()) {
        ROS_ERROR("Got an invalid path.");
        stop();
        pending_error_ = FollowPathResult::RESULT_STATUS_INTERNAL_ERROR;
        return;
    }

    if(is_running_) {
        if(goal.init_mode != FollowPathGoal::INIT_MODE_CONTINUE) {
            ROS_ERROR("got a new goal, stopping");
            stop();
        }
        is_running_ = false;
    }

    setPath(goal.path);

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

void PathFollower::publishPathMarker(){
    Eigen::Vector3d current_pose = getRobotPose();
    geometry_msgs::Point pt;
    pt.x = current_pose[0];
    pt.y = current_pose[1];
    g_robot_path_marker_.points.push_back(pt);

    g_points_pub_.publish(g_robot_path_marker_);
}
