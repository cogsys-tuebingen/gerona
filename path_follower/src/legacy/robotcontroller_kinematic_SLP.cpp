// HEADER
#include <path_follower/legacy/robotcontroller_kinematic_SLP.h>

// THIRD PARTY
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

// PROJECT
#include <path_follower/pathfollower.h>
#include <path_follower/utils/cubic_spline_interpolation.h>
#include <utils_general/MathHelper.h>
#include <cmath>

// SYSTEM
#include <deque>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;


RobotController_Kinematic_SLP::RobotController_Kinematic_SLP(PathFollower *path_driver):
    RobotController_Interpolation(path_driver),
    cmd_(this),
    view_direction_(LookInDrivingDirection),
    vn_(0.0),
    delta_(90.0*M_PI/180.0),
    Ts_(0.02),
    xe(0),
    ye(0),
    curv_sum_(1e-3),
    distance_to_goal_(1e-3),
    distance_to_obstacle_(1e-3)
{
    visualizer_ = Visualizer::getInstance();

    look_at_cmd_sub_ = nh_.subscribe<std_msgs::String>("/look_at/cmd", 10,
                                                       &RobotController_Kinematic_SLP::lookAtCommand, this);
    look_at_sub_ = nh_.subscribe<geometry_msgs::PointStamped>("/look_at", 10,
                                                              &RobotController_Kinematic_SLP::lookAt, this);

    laser_sub_front_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan/front/filtered", 10,
                                                             &RobotController_Kinematic_SLP::laserFront, this);
    laser_sub_back_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan/back/filtered", 10,
                                                            &RobotController_Kinematic_SLP::laserBack, this);

    lookInDrivingDirection();

}

void RobotController_Kinematic_SLP::stopMotion()
{

    cmd_.speed = 0;
    cmd_.direction_angle = 0;
    cmd_.rotation = 0;

    MoveCommand mcmd = cmd_;
    publishMoveCommand(mcmd);
}

void RobotController_Kinematic_SLP::lookAtCommand(const std_msgs::StringConstPtr &cmd)
{
    const std::string& command = cmd->data;

    if(command == "reset" || command == "view") {
        lookInDrivingDirection();
    } else if(command == "keep") {
        keepHeading();
    } else if(command == "rotate") {
        rotate();
    }
}

void RobotController_Kinematic_SLP::initialize()
{
    RobotController_Interpolation::initialize();

    // desired velocity
    vn_ = std::min(path_driver_->getOptions().max_velocity(), velocity_);
    ROS_WARN_STREAM("velocity_: " << velocity_ << ", vn: " << vn_);
}

void RobotController_Kinematic_SLP::lookAt(const geometry_msgs::PointStampedConstPtr &look_at)
{
    look_at_ = look_at->point;
    view_direction_ = LookAtPoint;
}

void RobotController_Kinematic_SLP::laserFront(const sensor_msgs::LaserScanConstPtr &scan)
{
    ranges_front_.clear();
    for(std::size_t i = 0, total = scan->ranges.size(); i < total; ++i) {
        float range = scan->ranges[i];
        if(range > scan->range_min && range < scan->range_max) {
            ranges_front_.push_back(range);
        }
    }
    findMinDistance();
}

void RobotController_Kinematic_SLP::laserBack(const sensor_msgs::LaserScanConstPtr &scan)
{
    ranges_back_.clear();
    for(std::size_t i = 0, total = scan->ranges.size(); i < total; ++i) {
        float range = scan->ranges[i];
        if(range > scan->range_min && range < scan->range_max) {
            ranges_back_.push_back(range);
        }
    }
    findMinDistance();
}

void RobotController_Kinematic_SLP::findMinDistance()
{
    std::vector<float> ranges;
    ranges.insert(ranges.end(), ranges_front_.begin(), ranges_front_.end());
    ranges.insert(ranges.end(), ranges_back_.begin(), ranges_back_.end());
    std::sort(ranges.begin(), ranges.end());

    if(ranges.size() <= 7) {
       distance_to_obstacle_ = 0;
        return;
    }

    distance_to_obstacle_ = ranges[0];

}

void RobotController_Kinematic_SLP::keepHeading()
{
    view_direction_ = KeepHeading;
}

void RobotController_Kinematic_SLP::rotate()
{
    view_direction_ = Rotate;
}

void RobotController_Kinematic_SLP::lookInDrivingDirection()
{
    view_direction_ = LookInDrivingDirection;
}

void RobotController_Kinematic_SLP::start()
{
    path_driver_->getCoursePredictor().reset();
}

RobotController::MoveCommandStatus RobotController_Kinematic_SLP::computeMoveCommand(MoveCommand *cmd)
{
    // omni drive can rotate.
    *cmd = MoveCommand(true);

    if(N_ < 2) {
        ROS_ERROR("[Line] path is too short (N = %d)", N_);

        stopMotion();
        return MoveCommandStatus::REACHED_GOAL;
    }

    // get the pose as pose(0) = x, pose(1) = y, pose(2) = theta
    Eigen::Vector3d current_pose = path_driver_->getRobotPose();

    double x_meas = current_pose[0];
    double y_meas = current_pose[1];
    double theta_meas = current_pose[2];
    //***//

    //find the orthogonal projection to the curve and extract the corresponding index

    double dist = 0;
    int ind = 0;
    double orth_proj = std::numeric_limits<double>::max();

    for (unsigned int i = 0; i < N_; i++){

        dist = hypot(x_meas - p_[i], y_meas - q_[i]);
        if(dist < orth_proj){

            orth_proj = dist;
            ind = i;

        }

    }

    //***//

    //find the slope of the desired path, and plot an orthogonal projection marker

    double theta_p = atan2(q_prim_[ind], p_prim_[ind]);

    visualization_msgs::Marker orth_proj_marker;
    orth_proj_marker.ns = "orth_proj";
    orth_proj_marker.header.frame_id = "/map";
    orth_proj_marker.header.stamp = ros::Time();
    orth_proj_marker.action = visualization_msgs::Marker::ADD;
    orth_proj_marker.id = 0;
    orth_proj_marker.color.r = 1;
    orth_proj_marker.color.g = 0;
    orth_proj_marker.color.b = 0;
    orth_proj_marker.color.a = 1.0;
    orth_proj_marker.scale.x = 0.1;
    orth_proj_marker.scale.y = 0.1;
    orth_proj_marker.scale.z = 0.5;
    orth_proj_marker.type = visualization_msgs::Marker::ARROW;

    geometry_msgs::Point from, to;
    from.x = x_meas;
    from.y = y_meas;
    to.x = p_[ind];
    to.y = q_[ind];

    orth_proj_marker.points.push_back(from);
    orth_proj_marker.points.push_back(to);

    visualizer_->getMarkerPublisher().publish(orth_proj_marker);

    //***//


    //***//

    //Calculate the look-ahead curvature

    //calculate the curvature, and stop when the look-ahead distance is reached (w.r.t. orthogonal projection)
    double s_cum_sum = 0;
    curv_sum_ = 1e-10;


    for (unsigned int i = ind + 1; i < N_; i++){

        s_cum_sum = s_[i] - s_[ind];
        curv_sum_ += fabs(curvature_[i]);

        if(s_cum_sum - opt_.look_ahead_dist() >= 0){
            break;
        }
    }


    //calculate the distance from the orthogonal projection to the goal, w.r.t. path
    distance_to_goal_ = s_[N_-1] - s_[ind];

    //get the robot's current angular velocity
    double angular_vel = path_driver_->getVelocity().angular.z;
    //***//


    //control

    ////ensure valid values
    if(distance_to_obstacle_ == 0 || !std::isfinite(distance_to_obstacle_)) distance_to_obstacle_ = 1e-10;
    if(distance_to_goal_ == 0 || !std::isfinite(distance_to_goal_)) distance_to_goal_ = 1e-10;

    double exponent = opt_.k_curv()*fabs(curv_sum_)
            + opt_.k_w()*fabs(angular_vel)
            + opt_.k_o()/distance_to_obstacle_
            + opt_.k_g()/distance_to_goal_;

    ////TODO: consider the minimum excitation speed
    cmd_.speed = std::max(0.2,vn_*exp(-exponent));

    cmd_.direction_angle = 0;

    cmd_.rotation = 0;

    //***//

    if (visualizer_->hasSubscriber()) {
        visualizer_->drawSteeringArrow(1, path_driver_->getRobotPoseMsg(), cmd_.direction_angle, 0.2, 1.0, 0.2);
    }

    //***//


    // check for end
    double distance_to_goal_eucl = hypot(x_meas - p_[N_-1], y_meas - q_[N_-1]);
    ROS_WARN_THROTTLE(1, "distance to goal: %f", distance_to_goal_eucl);

    if(distance_to_goal_eucl <= path_driver_->getOptions().goal_tolerance()) {
        return MoveCommandStatus::REACHED_GOAL;
    } else {
        *cmd = cmd_;

        return MoveCommandStatus::OKAY;
    }
}

void RobotController_Kinematic_SLP::publishMoveCommand(const MoveCommand &cmd) const
{
    geometry_msgs::Twist msg;
    msg.linear.x  = cmd.getVelocity();
    msg.linear.y  = 0;
    msg.angular.z = cmd.getRotationalVelocity();

    cmd_pub_.publish(msg);
}
