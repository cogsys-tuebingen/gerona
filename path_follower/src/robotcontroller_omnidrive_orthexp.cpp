/// THIRD PARTY
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

/// PROJECT
#include "pathfollower.h"
#include "behaviours.h"
#include "robotcontroller_omnidrive_orthexp.h"
#include "cubic_spline_interpolation.h"
#include "alglib/interpolation.h"
#include <utils_general/MathHelper.h>

/// SYSTEM
#include <deque>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;


RobotController_Omnidrive_OrthogonalExponential::RobotController_Omnidrive_OrthogonalExponential(ros::Publisher &cmd_publisher,
                                                                                                 PathFollower *path_driver):
    RobotController(cmd_publisher, path_driver),
    cmd_(this),
    nh_("~"),
    view_direction_(LookInDrivingDirection),
    initialized(false),
    vn(0.0),
    Ts(0.02),
    N(0),
    e_theta_curr(0),
    theta_des(90.0*M_PI/180.0),
    ///////////////////////////////
    max_curv_sum(0),
    curv_sum(0),
    look_ahead_dist(0.5),
    param_k_curv(0.05),
    param_k_g(0.4),
    param_k_o(1.0),
    param_k_w(0.5),
    distance_to_goal(0),
    distance_to_obstacle(1e5) //????
    /////////////////////////////

{
    visualizer_ = Visualizer::getInstance();
    interp_path_pub_ = nh_.advertise<nav_msgs::Path>("interp_path", 10);
    points_pub_ = nh_.advertise<visualization_msgs::Marker>("path_points", 10);

    look_at_cmd_sub_ = nh_.subscribe<std_msgs::String>("/look_at/cmd", 10,
                                                       boost::bind(&RobotController_Omnidrive_OrthogonalExponential::lookAtCommand, this, _1));
    look_at_sub_ = nh_.subscribe<geometry_msgs::PointStamped>("/look_at", 10,
                                                              boost::bind(&RobotController_Omnidrive_OrthogonalExponential::lookAt, this, _1));


    //control parameters
    nh_.param("k", param_k, 1.5);
    nh_.param("kp", param_kp, 0.4);
    nh_.param("kd", param_kd, 0.2);
    nh_.param("max_angular_velocity", max_angular_velocity, 2.0);

    // path marker
    robot_path_marker.header.frame_id = "map";
    robot_path_marker.header.stamp = ros::Time();
    robot_path_marker.ns = "my_namespace";
    robot_path_marker.id = 50;
    robot_path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    robot_path_marker.action = visualization_msgs::Marker::ADD;
    robot_path_marker.pose.position.x = 0;
    robot_path_marker.pose.position.y = 0;
    robot_path_marker.pose.position.z = 0;
    robot_path_marker.pose.orientation.x = 0.0;
    robot_path_marker.pose.orientation.y = 0.0;
    robot_path_marker.pose.orientation.z = 0.0;
    robot_path_marker.pose.orientation.w = 1.0;
    robot_path_marker.scale.x = 0.1;
    robot_path_marker.scale.y = 0.0;
    robot_path_marker.scale.z = 0.0;
    robot_path_marker.color.a = 1.0;
    robot_path_marker.color.r = 0.0;
    robot_path_marker.color.g = 0.0;
    robot_path_marker.color.b = 1.0;

    lookInDrivingDirection();

}

void RobotController_Omnidrive_OrthogonalExponential::publishCommand()
{
    if (!cmd_.isValid()) {
        ROS_FATAL("Invalid Command in RobotController_Omnidrive_OrthogonalExponential.");
        return;
    }

    geometry_msgs::Twist msg = cmd_;
    cmd_pub_.publish(msg);

    setFilteredSpeed(cmd_.speed);
}

void RobotController_Omnidrive_OrthogonalExponential::stopMotion()
{
    cmd_.speed = 0;
    cmd_.direction_angle = 0;
    cmd_.rotation = 0;

    publishCommand();
}

void RobotController_Omnidrive_OrthogonalExponential::lookAtCommand(const std_msgs::StringConstPtr &cmd)
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

void RobotController_Omnidrive_OrthogonalExponential::setPath(PathWithPosition path)
{
    RobotController::setPath(path);

    if(initialized) {
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

void RobotController_Omnidrive_OrthogonalExponential::lookAt(const geometry_msgs::PointStampedConstPtr &look_at)
{
    look_at_ = look_at->point;
    view_direction_ = LookAtPoint;
}

void RobotController_Omnidrive_OrthogonalExponential::keepHeading()
{
    view_direction_ = KeepHeading;
    theta_des = path_driver_->getRobotPose()[2];
}

void RobotController_Omnidrive_OrthogonalExponential::rotate()
{
    view_direction_ = Rotate;
}

void RobotController_Omnidrive_OrthogonalExponential::lookInDrivingDirection()
{
    view_direction_ = LookInDrivingDirection;
}

void RobotController_Omnidrive_OrthogonalExponential::initialize()
{
    // initialize the desired angle and the angle error
    e_theta_curr = path_driver_->getRobotPose()[2];

    // desired velocity
    vn = std::min(path_driver_->getOptions().max_velocity_, velocity_);
    ROS_WARN_STREAM("velocity_: " << velocity_ << ", vn: " << vn);
    initialized = true;
}

void RobotController_Omnidrive_OrthogonalExponential::clearBuffers()
{
    p.clear();
    q.clear();
    p_prim.clear();
    q_prim.clear();
    interp_path.poses.clear();
    robot_path_marker.points.clear();
    /////////////////////////////////////
    curvature.clear();
    /////////////////////////////////////
}

void RobotController_Omnidrive_OrthogonalExponential::interpolatePath()
{
    std::deque<Waypoint> waypoints;
    waypoints.insert(waypoints.end(), path_.current_path->begin(), path_.current_path->end());

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
    N = waypoints.size();

    if(N < 2) {
        return;
    }

    double X_arr[N], Y_arr[N], l_arr_unif[N];
    double f = std::max(0.0001, 1.0 / (double) (N-1));

    for(std::size_t i = 0; i < N; ++i) {
        const Waypoint& waypoint = waypoints[i];

        X_arr[i] = waypoint.x;
        Y_arr[i] = waypoint.y;
        l_arr_unif[i] = i * f;

    }

    //initialization before the interpolation
    alglib::real_1d_array X_alg, Y_alg, l_alg_unif;

    X_alg.setcontent(N,X_arr);
    Y_alg.setcontent(N,Y_arr);
    l_alg_unif.setcontent(N,l_arr_unif);

    alglib::spline1dinterpolant s_int1, s_int2;

    alglib::spline1dbuildcubic(l_alg_unif,X_alg,s_int1);
    alglib::spline1dbuildcubic(l_alg_unif,Y_alg,s_int2);

    //interpolate the path and find the derivatives, then publish the interpolated path
    for(uint i = 0; i < N; ++i) {
        double x_s = 0.0, y_s = 0.0, x_s_prim = 0.0, y_s_prim = 0.0, x_s_sek = 0.0, y_s_sek = 0.0;
        alglib::spline1ddiff(s_int1,l_alg_unif[i],x_s,x_s_prim,x_s_sek);
        alglib::spline1ddiff(s_int2,l_alg_unif[i],y_s,y_s_prim,y_s_sek);

        p.push_back(x_s);
        q.push_back(y_s);

        p_prim.push_back(x_s_prim);
        q_prim.push_back(y_s_prim);

        ////////////////////
        curvature.push_back((x_s_prim*x_s_sek - x_s_sek*y_s_prim)/
                            (sqrt((x_s_prim*x_s_prim + y_s_prim*y_s_prim)*(x_s_prim*x_s_prim + y_s_prim*y_s_prim)
                                  *(x_s_prim*x_s_prim + y_s_prim*y_s_prim))));
        ////////////////////
    }

    //////////////////////
    for(uint i = 0; i < N; ++i) {
        max_curv_sum += fabs(curvature[i]);
    }
    ////////////////////
}

void RobotController_Omnidrive_OrthogonalExponential::publishInterpolatedPath()
{
    if(N <= 2) {
        return;
    }

    for(uint i = 0; i < N; ++i) {
        geometry_msgs::PoseStamped poza;
        poza.pose.position.x = p[i];
        poza.pose.position.y = q[i];
        interp_path.poses.push_back(poza);
    }

    interp_path.header.frame_id = "map";
    interp_path_pub_.publish(interp_path);
}


void RobotController_Omnidrive_OrthogonalExponential::initOnLine()
{
    path_driver_->getCoursePredictor().reset();
}

void RobotController_Omnidrive_OrthogonalExponential::behaveOnLine()
{
    if(N < 2) {
        ROS_ERROR("[Line] path is too short");
        setStatus(path_msgs::FollowPathResult::MOTION_STATUS_SUCCESS);

        stopMotion();
        return;
    }

    Vector2d dir_of_mov = path_driver_->getCoursePredictor().smoothedDirection();
    if (!dir_of_mov.isZero() && path_driver_->isObstacleAhead(MathHelper::Angle(dir_of_mov))) {
        ROS_WARN_THROTTLE(1, "Collision!");
        //TODO: not so good to use result-constant if it is not finishing the action...
        setStatus(path_msgs::FollowPathResult::MOTION_STATUS_OBSTACLE);

        stopMotion();
        return;
    }

    // get the pose as pose(0) = x, pose(1) = y, pose(2) = theta
    Eigen::Vector3d current_pose = path_driver_->getRobotPose();

    double x_meas = current_pose[0];
    double y_meas = current_pose[1];
    double theta_meas = current_pose[2];
    //***//

    ROS_DEBUG("Theta: %f", theta_meas*180.0/M_PI);

    //find the orthogonal projection to the curve and extract the corresponding index

    double dist = 0;
    int ind = 0;
    double orth_proj = std::numeric_limits<double>::max();

    for (int i = 0; i < N; i++){

        dist = hypot(x_meas - p[i], y_meas - q[i]);
        if(dist < orth_proj){

            orth_proj = dist;
            ind = i;

        }

    }
    //***//

    //find the slope of the desired path, and plot a vector from the robot to the current point on the path

    double theta_p = atan2(q_prim[ind], p_prim[ind]);

    visualization_msgs::Marker marker;
    marker.ns = "orthexp";
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time();
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = 0;
    marker.color.r = 1;
    marker.color.g = 0;
    marker.color.b = 0;
    marker.color.a = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.5;
    marker.type = visualization_msgs::Marker::ARROW;

    geometry_msgs::Point from, to;
    from.x = x_meas;
    from.y = y_meas;
    to.x = p[ind];
    to.y = q[ind];

    marker.points.push_back(from);
    marker.points.push_back(to);

    visualizer_->getMarkerPublisher().publish(marker);

    //***//

    //determine the sign of the orthogonal distance
    static const double epsilon = 1e-3;
    if( ((theta_p > -M_PI/2) && (theta_p < M_PI/2) && (q[ind] > y_meas))
            || ((((theta_p > -M_PI) && (theta_p < -M_PI/2)) || ((theta_p > M_PI/2) && (theta_p < M_PI))
                 || (std::abs(theta_p - M_PI) < epsilon)) && (q[ind] < y_meas))
            || ((std::abs(theta_p + M_PI/2) < epsilon) && (p[ind] > x_meas))
            || ((std::abs(theta_p - M_PI/2) < epsilon) && (p[ind] < x_meas)) ){

        orth_proj *= -1;

    }

    ROS_DEBUG("Orthogonal distance: %f, theta_p: %f, theta_des: %f", orth_proj, theta_p*180.0/M_PI, theta_des*180.0/M_PI);

    //****//


    //check the "look-at" point, and calculate the rotation control

    switch(view_direction_) {
    case LookAtPoint:
        theta_des = std::atan2(look_at_.y - y_meas, look_at_.x - x_meas);
        break;
    case KeepHeading:
        // do nothing
        break;
    case LookInDrivingDirection:
        theta_des = cmd_.direction_angle + current_pose[2];//std::atan2(q[ind+1] - y_meas, p[ind+1] - x_meas);
        break;
    case Rotate:
        theta_des += 0.01;
        break;
    default:
        throw std::runtime_error("unknown view direction mode");
        break;
    }

    double e_theta_new = MathHelper::NormalizeAngle(theta_des - theta_meas);
    double e_theta_prim = (e_theta_new - e_theta_curr)/Ts;

    e_theta_curr = e_theta_new;

    //***//

    ////////////////////////////
    uint look_ahead_index;
    double look_ahead_difference = std::numeric_limits<double>::max();

    for (int i = ind; i < N; i++){

        if(fabs(hypot(x_meas - p[i], y_meas - q[i]) - look_ahead_dist) < look_ahead_difference){

            look_ahead_difference = fabs(hypot(x_meas - p[i], y_meas - q[i]) - look_ahead_dist);
            look_ahead_index = i;

        }
    }

    curv_sum = 1e-10;
    for (int i = ind; i <= look_ahead_index; i++){

        curv_sum += fabs(curvature[i]);
    }

    distance_to_goal = hypot(x_meas - p[N-1], y_meas - q[N-1]);

    double angular_vel = path_driver_->getVelocity().angular.z;

    ///////////////////////////

    //control

    double exponent = param_k_curv*fabs(curv_sum)
            + param_k_w*fabs(angular_vel)
            + param_k_o/distance_to_obstacle
            + param_k_g/distance_to_goal;

    cmd_.speed = vn*exp(-exponent);

    cmd_.direction_angle = atan(-param_k*orth_proj) + theta_p - theta_meas;

    cmd_.rotation = param_kp*e_theta_curr + param_kd*e_theta_prim;

    if(cmd_.rotation > max_angular_velocity) {
        cmd_.rotation = max_angular_velocity;
    } else if(cmd_.rotation < -max_angular_velocity) {
        cmd_.rotation = -max_angular_velocity;
    }

    //***//

    ///////////////////////////
    ROS_INFO("C_curv: %f, curv_sum: %f, max_curv_sum: %f, ind: %d, look_ahead_index: %d, vn: %f, v: %f",
             exp(-param_k_curv*1/curv_sum), curv_sum, max_curv_sum, ind, look_ahead_index, vn, cmd_.speed);
    ///////////////////////////


    ROS_DEBUG("alpha: %f, alpha_e: %f, e_theta_curr: %f",
              (atan(-param_k*orth_proj) + theta_p)*180.0/M_PI,
              atan(-param_k*orth_proj)*180.0/M_PI, e_theta_curr);

    if (visualizer_->hasSubscriber()) {
        visualizer_->drawSteeringArrow(1, path_driver_->getRobotPoseMsg(), cmd_.direction_angle, 0.2, 1.0, 0.2);
    }


    ////////////////////////////////////////////////
    geometry_msgs::Point pt;
    pt.x = x_meas;
    pt.y = y_meas;
    robot_path_marker.points.push_back(pt);

    points_pub_.publish(robot_path_marker);

    /////////////////////////////////////////////////

    // NULL PTR
    setStatus(path_msgs::FollowPathResult::MOTION_STATUS_MOVING);


    publishCommand();
}

void RobotController_Omnidrive_OrthogonalExponential::behaveAvoidObstacle()
{
    behaveOnLine();
}

bool RobotController_Omnidrive_OrthogonalExponential::behaveApproachTurningPoint()
{
    if(N < 2) {
        ROS_ERROR("[TurningPoint] path is too short");
        return true;
    }

    behaveOnLine();

    Eigen::Vector3d current_pose = path_driver_->getRobotPose();
    double x_meas = current_pose[0];
    double y_meas = current_pose[1];

    double distance_to_goal = hypot(x_meas - p[N-1], y_meas - q[N-1]);
    ROS_WARN_THROTTLE(1, "distance to goal: %f", distance_to_goal);

    return distance_to_goal <= path_driver_->getOptions().goal_tolerance_;
}

void RobotController_Omnidrive_OrthogonalExponential::reset()
{
    initialized = false;
}

