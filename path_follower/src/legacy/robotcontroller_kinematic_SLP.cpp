// HEADER
#include <path_follower/legacy/robotcontroller_kinematic_SLP.h>

// THIRD PARTY
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

// PROJECT
#include <path_follower/pathfollower.h>
#include <path_follower/utils/cubic_spline_interpolation.h>
#include "../alglib/interpolation.h"
#include <utils_general/MathHelper.h>
#include <cmath>

// SYSTEM
#include <deque>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;


RobotController_Kinematic_SLP::RobotController_Kinematic_SLP(PathFollower *path_driver):
    RobotController(path_driver),
    cmd_(this),
    nh_("~"),
    view_direction_(LookInDrivingDirection),
    initialized_(false),
    vn_(0.0),
    delta_(90.0*M_PI/180.0),
    N_(0),
    Ts_(0.02),
    s_prim_(0),
    xe(0),
    ye(0),
    curv_sum_(1e-3),
    distance_to_goal_(1e-3),
    distance_to_obstacle_(1e-3)
{
    visualizer_ = Visualizer::getInstance();
    interp_path_pub_ = nh_.advertise<nav_msgs::Path>("interp_path", 10);
    points_pub_ = nh_.advertise<visualization_msgs::Marker>("path_points", 10);

    look_at_cmd_sub_ = nh_.subscribe<std_msgs::String>("/look_at/cmd", 10,
                                                       &RobotController_Kinematic_SLP::lookAtCommand, this);
    look_at_sub_ = nh_.subscribe<geometry_msgs::PointStamped>("/look_at", 10,
                                                              &RobotController_Kinematic_SLP::lookAt, this);

    laser_sub_front_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan/front/filtered", 10,
                                                             &RobotController_Kinematic_SLP::laserFront, this);
    laser_sub_back_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan/back/filtered", 10,
                                                            &RobotController_Kinematic_SLP::laserBack, this);

    // path marker
    robot_path_marker_.header.frame_id = "map";
    robot_path_marker_.header.stamp = ros::Time();
    robot_path_marker_.ns = "my_namespace";
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
    robot_path_marker_.scale.x = 0.1;
    robot_path_marker_.scale.y = 0.0;
    robot_path_marker_.scale.z = 0.0;
    robot_path_marker_.color.a = 1.0;
    robot_path_marker_.color.r = 0.0;
    robot_path_marker_.color.g = 0.0;
    robot_path_marker_.color.b = 1.0;

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

void RobotController_Kinematic_SLP::setPath(Path::Ptr path)
{
    RobotController::setPath(path);

    if(initialized_) {
        return;
    }

    clearBuffers();

    try {
        interpolatePath();
        publishInterpolatedPath();

    } catch(const alglib::ap_error& error) {
        throw std::runtime_error(error.msg);
    }

    initialize();
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

void RobotController_Kinematic_SLP::initialize()
{
    // desired velocity
    vn_ = std::min(path_driver_->getOptions().max_velocity(), velocity_);
    ROS_WARN_STREAM("velocity_: " << velocity_ << ", vn: " << vn_);
    initialized_ = true;
}

void RobotController_Kinematic_SLP::clearBuffers()
{
    p_.clear();
    q_.clear();
    p_prim_.clear();
    q_prim_.clear();
    s_.clear();
    interp_path_.poses.clear();
    robot_path_marker_.points.clear();
    curvature_.clear();

}

void RobotController_Kinematic_SLP::interpolatePath()
{
    std::deque<Waypoint> waypoints;
    waypoints.insert(waypoints.end(), path_->getCurrentSubPath().begin(), path_->getCurrentSubPath().end());

    // (messy) hack!!!!!
    // remove waypoints that are closer than 0.1 meters to the starting point
    Waypoint start = waypoints.front();
    while(!waypoints.empty()) {
        std::deque<Waypoint>::iterator it = waypoints.begin();
        const Waypoint& wp = *it;

        double dx = wp.x - start.x;
        double dy = wp.y - start.y;
        double distance = hypot(dx, dy);
        if(distance < 0.1) {
            waypoints.pop_front();
        } else {
            break;
        }
    }

    //copy the waypoints to arrays X_arr and Y_arr, and introduce a new array l_arr_unif required for the interpolation
    //as an intermediate step, calculate the arclength of the curve, and do the reparameterization with respect to arclength

    N_ = waypoints.size();

    if(N_ < 2) {
        return;
    }

    double X_arr[N_], Y_arr[N_], l_arr_unif[N_];
    double L = 0; 

    for(std::size_t i = 0; i < N_; ++i) {
        const Waypoint& waypoint = waypoints[i];

        X_arr[i] = waypoint.x;
        Y_arr[i] = waypoint.y;

    }

    for(std::size_t i = 1; i < N_; i++){

        L += hypot(X_arr[i] - X_arr[i-1], Y_arr[i] - Y_arr[i-1]);

    }
    ROS_INFO("Length of the path: %lf m", L);


    double f = std::max(0.0001, L / (double) (N_-1));

    for(std::size_t i = 1; i < N_; i++){

        l_arr_unif[i] = i * f;

    }

    //initialization before the interpolation
    alglib::real_1d_array X_alg, Y_alg, l_alg_unif;

    X_alg.setcontent(N_,X_arr);
    Y_alg.setcontent(N_,Y_arr);
    l_alg_unif.setcontent(N_,l_arr_unif);

    alglib::spline1dinterpolant s_int1, s_int2;

    alglib::spline1dbuildcubic(l_alg_unif,X_alg,s_int1);
    alglib::spline1dbuildcubic(l_alg_unif,Y_alg,s_int2);

    //interpolate the path and find the derivatives, then publish the interpolated path
    double test_cum_sum = 0;
    for(uint i = 0; i < N_; ++i) {
        double x_s = 0.0, y_s = 0.0, x_s_prim = 0.0, y_s_prim = 0.0, x_s_sek = 0.0, y_s_sek = 0.0;
        alglib::spline1ddiff(s_int1,l_alg_unif[i],x_s,x_s_prim,x_s_sek);
        alglib::spline1ddiff(s_int2,l_alg_unif[i],y_s,y_s_prim,y_s_sek);

        p_.push_back(x_s);
        q_.push_back(y_s);

        p_prim_.push_back(x_s_prim);
        q_prim_.push_back(y_s_prim);

        s_.push_back(l_alg_unif[i]);

        curvature_.push_back((x_s_prim*y_s_sek - x_s_sek*y_s_prim)/
                            (sqrt(pow((x_s_prim*x_s_prim + y_s_prim*y_s_prim), 3))));

        ///test
        if(i>0)
        test_cum_sum += hypot(p_[i] - p_[i-1], q_[i] - q_[i-1]);
        ROS_INFO("cum_sum: %lf", test_cum_sum);
        ROS_INFO("s: %lf", s_[i]);
        ROS_INFO("x-difference: %lf, y-difference: %lf", fabs(p_[i]-X_arr[i]), fabs(q_[i]-Y_arr[i])); //WTF??

    }

}

void RobotController_Kinematic_SLP::publishInterpolatedPath()
{
    if(N_ <= 2) {
        return;
    }

    for(uint i = 0; i < N_; ++i) {
        geometry_msgs::PoseStamped poza;
        poza.pose.position.x = p_[i];
        poza.pose.position.y = q_[i];
        interp_path_.poses.push_back(poza);
    }

    interp_path_.header.frame_id = "map";
    interp_path_pub_.publish(interp_path_);
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
    ////make sure there are no problems with vector range
    if(ind == 0) ind = 1;
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
    double look_ahead_cum_sum = 0;
    curv_sum_ = 1e-10;


    for (unsigned int i = ind + 1; i < N_; i++){

        look_ahead_cum_sum += hypot(p_[i] - p_[i-1], q_[i] - q_[i-1]);
        curv_sum_ += fabs(curvature_[i]);


        if(look_ahead_cum_sum - opt_.look_ahead_dist() >= 0){
            break;
        }
    }


    //calculate the distance from the orthogonal projection to the goal, w.r.t. path

    distance_to_goal_ = 1e-3;
    for (unsigned int i = ind + 1; i < N_; i++){

        distance_to_goal_ += hypot(p_[i] - p_[i-1], q_[i] - q_[i-1]);

    }


    double angular_vel = path_driver_->getVelocity().angular.z;
    //***//


    //control

    ////ensure valid values
    if(distance_to_obstacle_ == 0 || !std::isfinite(distance_to_obstacle_)) distance_to_obstacle_ = 1e-3;
    if(distance_to_goal_ == 0 || !std::isfinite(distance_to_goal_)) distance_to_goal_ = 1e-3;

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


    //Vizualize the path driven by the robot
    geometry_msgs::Point pt;
    pt.x = x_meas;
    pt.y = y_meas;
    robot_path_marker_.points.push_back(pt);

    points_pub_.publish(robot_path_marker_);
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

void RobotController_Kinematic_SLP::reset()
{
    initialized_ = false;
}

