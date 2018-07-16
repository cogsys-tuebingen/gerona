#include <path_follower/controller/robotcontroller.h>

/// PROJECT
#include <path_follower/pathfollower.h>
#include <cslibs_utils/MathHelper.h>
#include <path_follower/utils/path_exceptions.h>
#include <path_follower/utils/pose_tracker.h>
#include <path_follower/utils/visualizer.h>
#include <path_follower/collision_avoidance/collision_avoider.h>
#include <path_follower/utils/obstacle_cloud.h>

///SYSTEM
#include <pcl_ros/point_cloud.h>

RobotController::RobotController()
    : pnh_("~"),
      pose_tracker_(nullptr),
      collision_avoider_(nullptr),
      visualizer_(Visualizer::getInstance()),
      velocity_(0.0f),
      dir_sign_(1.0f),
      interpolated_(false),
      proj_ind_(0),
      k_curv_(0.0),
      k_o_(0.0),
      k_g_(0.0),
      k_w_(0.0),
      look_ahead_dist_(0.0),
      obst_threshold_(0.0),
      curv_sum_(0.0)
{
    orth_proj_ = std::numeric_limits<double>::max();
    distance_to_goal_ = 1e5;
    distance_to_obstacle_ = 1e5;

    initPublisher(&cmd_pub_);

    points_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    interp_path_pub_ = nh_.advertise<nav_msgs::Path>("interp_path", 10);
    exp_control_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("exp_control_parameters", 10);

    // path marker
    robot_path_marker_.header.frame_id = getFixedFrame();
    robot_path_marker_.header.stamp = ros::Time();
    robot_path_marker_.ns = "robot path";
    robot_path_marker_.id = 50;
    robot_path_marker_.type = visualization_msgs::Marker::LINE_STRIP;
    robot_path_marker_.action = visualization_msgs::Marker::ADD;
    robot_path_marker_.pose.position.x = 0;
    robot_path_marker_.pose.position.y = 0;
    robot_path_marker_.pose.position.z = 0;
    robot_path_marker_.pose.orientation.x = 0.0;
    robot_path_marker_.pose.orientation.y = 0.0;
    robot_path_marker_.pose.orientation.z = 0.0;
    robot_path_marker_.pose.orientation.w = 1.0;
    robot_path_marker_.scale.x = 0.01;
    robot_path_marker_.scale.y = 0.0;
    robot_path_marker_.scale.z = 0.0;
    robot_path_marker_.color.a = 1.0;
    robot_path_marker_.color.r = 0.0;
    robot_path_marker_.color.g = 0.0;
    robot_path_marker_.color.b = 1.0;
}

void RobotController::init(PoseTracker *pose_tracker, CollisionAvoider *collision_avoider)
{
    pose_tracker_ = pose_tracker;
    collision_avoider_ = collision_avoider;
}

std::string RobotController::getFixedFrame() const
{
    if(path_) {
        return path_->getFrameId();
    } else {
        return PathFollowerParameters::getInstance()->world_frame();
    }
}

void RobotController::computeMovingDirection()
{
    ROS_INFO_STREAM("path length " << path_->subPathCount());
    // decide whether to drive forward or backward
    if (path_->getCurrentSubPath().forward) {
        setDirSign(1.f);
        ROS_INFO_STREAM("following forwards path segment of length " << path_->getCurrentSubPath().size());
    } else {
        setDirSign(-1.f);
        ROS_WARN_STREAM("following backwards path segment of length " << path_->getCurrentSubPath().size());
    }
}

void RobotController::initialize()
{
    interpolated_ = true;
    //reset the index of the orthogonal projection
    proj_ind_ = 0;
}

void RobotController::reset()
{
    //reset the interpolation
    interpolated_ = false;
    //reset the index of the orthogonal projection
    proj_ind_ = 0;

    //reset the parameters for the exponential speed control
    k_curv_ = getParameters().k_curv();
    k_o_ = getParameters().k_o();
    k_g_ = getParameters().k_g();
    k_w_ = getParameters().k_w();
    look_ahead_dist_ = getParameters().look_ahead_dist();
    obst_threshold_ = getParameters().obst_threshold();

}

void RobotController::publishInterpolatedPath()
{
    interp_path_pub_.publish((nav_msgs::Path) path_interpol);
}

bool RobotController::reachedGoal(const Eigen::Vector3d& pose) const
{

    const unsigned int end = path_interpol.n() - 1;
    return hypot(path_interpol.p(end) - pose[0], path_interpol.q(end) - pose[1])
            <= getParameters().goal_tolerance();
}

void RobotController::setPath(Path::Ptr path)
{
    if (path->subPathCount() == 0) {
        throw EmergencyBreakException("No sub paths",
                                      path_msgs::FollowPathResult::RESULT_STATUS_TF_FAIL);
    }

    path_ = path;

    robot_path_marker_.points.clear();

    geometry_msgs::PoseStamped wp_pose;
    wp_pose.header.stamp = ros::Time::now();
    wp_pose.pose = path->getCurrentWaypoint();
    if ( !pose_tracker_->transformToLocal( wp_pose, next_wp_local_)) {
        throw EmergencyBreakException("cannot transform path",
                                      path_msgs::FollowPathResult::RESULT_STATUS_TF_FAIL);
    }

    computeMovingDirection();

    if(!interpolated_) {
        path_interpol.interpolatePath(path);
        publishInterpolatedPath();

        initialize();
    }

    //reset the index of the orthogonal projection
    proj_ind_ = 0;

}

void RobotController::setGlobalPath(Path::Ptr path)
{
    //global_path_ = path;
    global_path_.interpolatePath(path, true);
}

void RobotController::initPublisher(ros::Publisher *pub) const
{
    ros::NodeHandle nh;
    //TODO: implement a dynamic switching between velocity and torque mode
    //torque mode
    //*pub = nh.advertise<std_msgs::Float64MultiArray>("wheel_torques", 10);
    //velocity mode
    *pub = nh.advertise<geometry_msgs::Twist> ("cmd_vel", 10);
}

double RobotController::calculateAngleError()
{
    geometry_msgs::Pose waypoint   = path_->getCurrentWaypoint();
    geometry_msgs::Pose robot_pose = pose_tracker_->getRobotPoseMsg();
    return MathHelper::AngleClamp(tf::getYaw(waypoint.orientation) - tf::getYaw(robot_pose.orientation));
}

RobotController::ControlStatus RobotController::MCS2CS(RobotController::MoveCommandStatus s)
{
    switch (s) {
    case MoveCommandStatus::OKAY:
        return ControlStatus::OKAY;
    case MoveCommandStatus::REACHED_GOAL:
        return ControlStatus::REACHED_GOAL;
    default:
        ROS_ERROR("MoveCommandStatus %d is not handled by MCS2CS! Return ERROR instead.", (int) s);
    case MoveCommandStatus::ERROR:
        return ControlStatus::ERROR;
    }
}

void RobotController::publishPathMarker()
{
    Eigen::Vector3d current_pose = pose_tracker_->getRobotPose();
    geometry_msgs::Point pt;
    pt.x = current_pose[0];
    pt.y = current_pose[1];
    robot_path_marker_.points.push_back(pt);

    points_pub_.publish(robot_path_marker_);
}

void RobotController::findOrthogonalProjection()
{
    //find the orthogonal projection to the curve and extract the corresponding index

    orth_proj_ = std::numeric_limits<double>::max();

    Eigen::Vector3d current_pose = pose_tracker_->getRobotPose();
    double x_meas = current_pose[0];
    double y_meas = current_pose[1];

    double dist = 0;
    double dx = 0.0;
    double dy = 0.0;
    //this is a trick for closed paths, if the start and goal point are very close
    //without this, the robot would reach the goal, without even driving
    int old_ind = proj_ind_;

    for (unsigned int i = proj_ind_, n = path_interpol.n(); i < n; i++){

        dist = hypot(x_meas - path_interpol.p(i), y_meas - path_interpol.q(i));
        if((dist < orth_proj_) && (i - old_ind <= 3)){

            orth_proj_ = dist;
            proj_ind_ = i;

            dx = x_meas - path_interpol.p(proj_ind_);
            dy = y_meas - path_interpol.q(proj_ind_);
        }

        // debuging information
        if(i == n - 1) {
            if(proj_ind_ != i) {
                ROS_DEBUG_STREAM("projection: dist:" << dist
                                 << ", orth_proj: " << orth_proj_
                                 << ", old_ind: " << old_ind);
            }
        }
    }

    //determine the sign of the orthogonal distance
    Eigen::Vector2d path2vehicle_vec(dx, dy);
    double path2vehicle_angle = MathHelper::Angle(path2vehicle_vec);
    double theta_diff = MathHelper::AngleDelta(path_interpol.theta_p(proj_ind_), path2vehicle_angle);

    if( theta_diff < 0 && theta_diff >= -M_PI){

        orth_proj_ = -fabs(orth_proj_);

    }else{
        orth_proj_ = fabs(orth_proj_);
    }
    //***//
}


bool RobotController::isGoalReached(MoveCommand *cmd)
{
    Eigen::Vector3d current_pose = pose_tracker_->getRobotPose();
    double x_meas = current_pose[0];
    double y_meas = current_pose[1];

    ROS_DEBUG_STREAM("checking goal reached: projected index: " << proj_ind_
                     << ", last index: " << path_interpol.n()-1);

    // check for the subpaths, and see if the goal is reached
    if(proj_ind_ == path_interpol.n()-1) {
        path_->switchToNextSubPath();
        // check if we reached the actual goal or just the end of a subpath
        if (path_->isDone()) {

            MoveCommand cmd_stop(true);
            cmd_stop.setVelocity(0.0);
            cmd_stop.setDirection(0.0);
            cmd_stop.setRotationalVelocity(0.0);

            *cmd = cmd_stop;

            double distance_to_goal_eucl = hypot(x_meas - path_interpol.p(path_interpol.n()-1),
                                                 y_meas - path_interpol.q(path_interpol.n()-1));

            ROS_INFO_THROTTLE(1, "Final positioning error: %f m", distance_to_goal_eucl);
            return true;

        } else {
            //reset the orthogonal projection
            proj_ind_ = 0;

            ROS_INFO("Next subpath...");
            // interpolate the next subpath
            path_interpol.interpolatePath(path_);
            publishInterpolatedPath();

            // recompute the driving direction
            computeMovingDirection();

            return false;
        }
    }
    return false;
}

double RobotController::exponentialSpeedControl()
{

    //compute the curvature, and stop when the look-ahead distance is reached (w.r.t. orthogonal projection)
    double s_cum_sum = 0;
    curv_sum_ = 0.0;

    for (unsigned int i = proj_ind_ + 1; i < path_interpol.n(); i++){

        s_cum_sum = path_interpol.s(i) - path_interpol.s(proj_ind_);
        curv_sum_ += std::abs(path_interpol.curvature(i));

        if(s_cum_sum - look_ahead_dist_ >= 0){
            break;
        }
    }

    //compute the distance from the orthogonal projection to the goal, w.r.t. path
    distance_to_goal_ = path_interpol.s(path_interpol.n()-1) - path_interpol.s(proj_ind_);

    //get the robot's current angular velocity
    double angular_vel = pose_tracker_->getVelocity().angular.z;

    double obst_angle = 0.0;

    double min_dist = std::numeric_limits<double>::infinity();
    if(collision_avoider_->hasObstacles()) {
        auto obstacle_cloud = collision_avoider_->getObstacles();
        const pcl::PointCloud<pcl::PointXYZ>& cloud = *obstacle_cloud->cloud;
        if(cloud.header.frame_id == "base_link" || cloud.header.frame_id == "/base_link") {
            for(const pcl::PointXYZ& pt : cloud) {

                if(std::sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z) < min_dist){

                    obst_angle = std::atan2(pt.y, pt.x);
                }
                min_dist = std::min<double>(min_dist, std::sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z));
            }

        } else {
            tf::Transform trafo = pose_tracker_->getTransform(pose_tracker_->getRobotFrameId(), cloud.header.frame_id, ros::Time(0), ros::Duration(0));
            for(const pcl::PointXYZ& pt : cloud) {

                tf::Point pt_cloud(pt.x, pt.y, pt.z);
                tf::Point pt_robot = trafo * pt_cloud;

                if(pt_robot.length() < min_dist){
                    obst_angle = std::atan2(pt_robot.getY(), pt_robot.getX());
                }

                min_dist = std::min<double>(min_dist, pt_robot.length());
            }
        }
    }

    distance_to_obstacle_ = min_dist;

    //ensure valid values
    if (!std::isnormal(distance_to_obstacle_)) distance_to_obstacle_ = 1e5;
    if (!std::isnormal(distance_to_goal_)) distance_to_goal_ = 1e5;

    //consider only the obstacles closer than a threshold, and slow down only when driving forward
    double epsilon_o = 0.0;
    if(distance_to_obstacle_ <= obst_threshold_){
        epsilon_o = k_o_/distance_to_obstacle_;
    }
    //consider the obstacle orientation for backward driving
    if(getDirSign() < 0){
        obst_angle -= M_PI;
    }

    double fact_curv = k_curv_*curv_sum_;
    if (!std::isnormal(fact_curv)) fact_curv = 0.0;
    double fact_w = k_w_*fabs(angular_vel);
    if (!std::isnormal(fact_w)) fact_w = 0.0;
    double fact_obst = epsilon_o*std::max(0.0, cos(obst_angle));
    if (!std::isnormal(fact_obst)) fact_obst = 0.0;
    double fact_goal = std::min(3.0, k_g_/distance_to_goal_); // TODO: remove this hack to avoid non-moving robot!
    if (!std::isnormal(fact_goal)) fact_goal = 0.0;

    //publish the factors of the exponential speed control
    std_msgs::Float64MultiArray exp_control_array;
    exp_control_array.data.resize(4);
    exp_control_array.data[0] = fact_curv;
    exp_control_array.data[1] = fact_w;
    exp_control_array.data[2] = fact_obst;
    exp_control_array.data[3] = fact_goal;
    exp_control_pub_.publish(exp_control_array);

//    ROS_INFO("k_curv: %f, k_o: %f, k_w: %f, k_g: %f, look_ahead: %f, obst_thresh: %f",
//             k_curv_, k_o_, k_w_, k_g_, look_ahead_dist_, obst_threshold_);
    double exponent = fact_curv + fact_w + fact_obst + fact_goal;
    return exp(-exponent);
}

RobotController::ControlStatus RobotController::execute()
{
    if(!path_) {
       return ControlStatus::ERROR;
    }

    publishPathMarker();

    MoveCommand cmd;
    MoveCommandStatus status = computeMoveCommand(&cmd);


    if (status != MoveCommandStatus::OKAY) {
        stopMotion();
        return MCS2CS(status);
    } else {
        CollisionAvoider::State state(path_, *PathFollowerParameters::getInstance());
        bool cmd_modified = collision_avoider_->avoid(&cmd, state);

        if (!cmd.isValid()) {
            ROS_ERROR("Invalid move command.");
            stopMotion();
            return ControlStatus::ERROR;
        } else {
            publishMoveCommand(cmd);
            return cmd_modified ? ControlStatus::OBSTACLE : ControlStatus::OKAY;
        }
    }
}
