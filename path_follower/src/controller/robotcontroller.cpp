#include <path_follower/controller/robotcontroller.h>

/// PROJECT
#include <path_follower/pathfollower.h>
#include <cslibs_utils/MathHelper.h>
#include <path_follower/utils/path_exceptions.h>
#include <path_follower/utils/pose_tracker.h>
#include <path_follower/utils/visualizer.h>
#include <path_follower/collision_avoidance/collision_avoider.h>

/// THIRD PARTY

RobotController::RobotController()
    : pnh_("~"),
      pose_tracker_(nullptr),
      collision_avoider_(nullptr),
      global_opt_(nullptr),
      visualizer_(Visualizer::getInstance()),
      velocity_(0.0f),
      dir_sign_(1.0f),
      interpolated_(false)
{
    orth_proj_ = std::numeric_limits<double>::max();

    initPublisher(&cmd_pub_);

    points_pub_ = nh_.advertise<visualization_msgs::Marker>("path_points", 10);
    interp_path_pub_ = nh_.advertise<nav_msgs::Path>("interp_path", 10);

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

void RobotController::init(PoseTracker *pose_tracker, CollisionAvoider *collision_avoider, const PathFollowerParameters *options)
{
    pose_tracker_ = pose_tracker;
    collision_avoider_ = collision_avoider;
    global_opt_ = options;
}

std::string RobotController::getFixedFrame() const
{
    if(path_) {
        return path_->getFrameId();
    } else {
        return "map";
    }
}

void RobotController::computeMovingDirection()
{
    // decide whether to drive forward or backward
    if (path_->getCurrentSubPath().forward) {
        setDirSign(1.f);
    } else {
        setDirSign(-1.f);
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
    interpolated_ = false;
    //reset the index of the orthogonal projection
    proj_ind_ = 0;

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
        std::cerr << "interpolating path in frame " << path->getFrameId() << std::endl;

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
    //*pub = nh.advertise<std_msgs::Float64MultiArray>("/wheel_torques", 10);
    //velocity mode
    *pub = nh.advertise<geometry_msgs::Twist> ("/cmd_vel", 10);
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
        ROS_ERROR("MoveCommandStatus %d is not handled by MCS2CS! Return ERROR instead.", s);
    case MoveCommandStatus::ERROR:
        return ControlStatus::ERROR;
    }
}

bool RobotController::isOmnidirectional() const
{
    return false;
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

    for (unsigned int i = proj_ind_; i < path_interpol.n(); i++){

        dist = hypot(x_meas - path_interpol.p(i), y_meas - path_interpol.q(i));
        if((dist < orth_proj_) && (i - old_ind <= 3)){

            orth_proj_ = dist;
            proj_ind_ = i;

            dx = x_meas - path_interpol.p(proj_ind_);
            dy = y_meas - path_interpol.q(proj_ind_);

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

    // check for the subpaths, and see if the goal is reached
    if(proj_ind_ == path_interpol.n()-1) {
        path_->switchToNextSubPath();
        // check if we reached the actual goal or just the end of a subpath
        if (path_->isDone()) {

            MoveCommand cmd_stop;
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
        CollisionAvoider::State state(path_, *global_opt_);
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
