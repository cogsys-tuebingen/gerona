/// THIRD PARTY
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

/// PROJECT
#include "pathfollower.h"
#include "behaviours.h"
#include "robotcontroller_omnidrive_vv.h"
#include "cubic_spline_interpolation.h"
#include "alglib/interpolation.h"

using namespace Eigen;


RobotController_Omnidrive_VirtualVehicle::RobotController_Omnidrive_VirtualVehicle(ros::Publisher &cmd_publisher,
                                                                                   PathFollower *path_driver):
    RobotController(cmd_publisher, path_driver),
    cmd_(this),
    nh_("~"),
    initialized(false),
    N(0),
    vn(0.0),
    counter(0),
    d(0.5),
    k(1.5),
    epsilon(d),
    ro(d),
    alpha(0.1),
    c(exp(alpha*d)),
    gama(vn/d),
    Ts(0.02),
    psi_d_prev(0.0)


{
    visualizer_ = Visualizer::getInstance();
    interp_path_pub_ = nh_.advertise<nav_msgs::Path>("interp_path", 10);
    vv_path_pub_ = nh_.advertise<nav_msgs::Path>("vv_path", 10);
    points_pub_ = nh_.advertise<visualization_msgs::Marker>("path_points", 10);

}

void RobotController_Omnidrive_VirtualVehicle::publishCommand()
{
    if (!cmd_.isValid()) {
        ROS_FATAL("Invalid Command in RobotController_Omnidrive_VirtualVehicle.");
        return;
    }

    geometry_msgs::Twist msg = cmd_;
    cmd_pub_.publish(msg);

    setFilteredSpeed(cmd_.speed);
}

void RobotController_Omnidrive_VirtualVehicle::stopMotion()
{
    cmd_.speed = 0;
    cmd_.direction_angle = 0;
    cmd_.rotation = 0;

    publishCommand();
}

void RobotController_Omnidrive_VirtualVehicle::setPath(PathWithPosition path)
{
    RobotController::setPath(path);

    if(initialized) {

        return;
    }


    initialized = true;

    const std::vector<Waypoint>& waypoints = *path_.current_path;

    std::vector<double> X, Y;

    for(std::size_t i = 0; i < waypoints.size(); ++i) {
        const Waypoint& waypoint = waypoints[i];

        X.push_back(waypoint.x);
        Y.push_back(waypoint.y);

    }

    std::vector<double> l_cum(X.size());
    l_cum[0] = 0;
    double L = 0;
    for(std::size_t i = 1; i < X.size(); ++i) {

        L += hypot(X[i] - X[i-1], Y[i] - Y[i-1]);
        l_cum[i] = L;

    }

    ROS_DEBUG("L: %f", L);

    vn = path_driver_->getOptions().max_velocity_; //desired velocity

    c = exp(alpha*d);
    gama = vn/d;

    N = ceil(L / (vn * Ts) + 1);



    double f = std::max(0.0001, L / (double) (N-1));
    std::vector<double> l_unif(N);
    l_unif.clear();
    l_unif.resize(N, 0);
    for(uint i=0; i<N; i++){

        l_unif[i] = i * f;
    }


    double X_arr[X.size()], Y_arr[X.size()], l_arr_cum[X.size()], l_arr_unif[N];
    alglib::real_1d_array X_pom, Y_pom, l_pom_cum, l_pom_unif;

    for(uint i=0; i<X.size(); i++){

        X_arr[i] = X[i];
        Y_arr[i] = Y[i];
        l_arr_cum[i] = l_cum[i];

    }

    for(uint i=0; i<N; i++){

        l_arr_unif[i] = l_unif[i];
    }

    X_pom.setcontent(X.size(),X_arr);
    Y_pom.setcontent(X.size(),Y_arr);
    l_pom_cum.setcontent(X.size(),l_arr_cum);
    l_pom_unif.setcontent(N,l_arr_unif);


    double x_s = 0.0, y_s = 0.0, x_s_prim = 0.0, y_s_prim = 0.0, x_s_sek = 0.0, y_s_sek = 0.0;


    alglib::spline1dinterpolant s_int1, s_int2;
    alglib::spline1dbuildcubic(l_pom_cum,X_pom,s_int1);
    alglib::spline1dbuildcubic(l_pom_cum,Y_pom,s_int2);

    p.clear();
    q.clear();
    p_prim.clear();
    q_prim.clear();
    interp_path.poses.clear();

    for(uint i = 0; i < N; ++i) {
        geometry_msgs::PoseStamped poza;
        alglib::spline1ddiff(s_int1,l_pom_unif[i],x_s,x_s_prim,x_s_sek);
        alglib::spline1ddiff(s_int2,l_pom_unif[i],y_s,y_s_prim,y_s_sek);
        poza.pose.position.x = x_s;
        poza.pose.position.y = y_s;
        interp_path.poses.push_back(poza);

        p.push_back(poza.pose.position.x);
        q.push_back(poza.pose.position.y);

        p_prim.push_back(x_s_prim);
        q_prim.push_back(y_s_prim);

    }

    interp_path.header.frame_id = "map";
    interp_path_pub_.publish(interp_path);

    double step = L/N;
    ROS_DEBUG("Step: %f", step);

    xr_calc.clear();
    yr_calc.clear();

    xr_calc.resize(N, p[0]);
    yr_calc.resize(N, q[0]);
    ROS_DEBUG("p[0]: %f, q[0]: %f",p[0],q[0]);
    vv_path.poses.clear();

    /////////////////////////////////////////////////
    robot_path.points.clear();
    robot_path.header.frame_id = "map";
    robot_path.header.stamp = ros::Time();
    robot_path.ns = "my_namespace";
    robot_path.id = 50;
    robot_path.type = visualization_msgs::Marker::LINE_STRIP;
    robot_path.action = visualization_msgs::Marker::ADD;
    robot_path.pose.position.x = 0;
    robot_path.pose.position.y = 0;
    robot_path.pose.position.z = 0;
    robot_path.pose.orientation.x = 0.0;
    robot_path.pose.orientation.y = 0.0;
    robot_path.pose.orientation.z = 0.0;
    robot_path.pose.orientation.w = 1.0;
    robot_path.scale.x = 0.1;
    robot_path.scale.y = 0.0;
    robot_path.scale.z = 0.0;
    robot_path.color.a = 1.0;
    robot_path.color.r = 0.0;
    robot_path.color.g = 0.0;
    robot_path.color.b = 1.0;
    /////////////////////////////////////////////////

    counter = 0;

}

void RobotController_Omnidrive_VirtualVehicle::initOnLine()
{


}

void RobotController_Omnidrive_VirtualVehicle::behaveOnLine()
{
    // pose:
    Eigen::Vector3d current_pose = path_driver_->getRobotPose();
    //pose(0) = x, pose(1) = y, pose(2) = theta


    double x = current_pose[0];
    double y = current_pose[1];
    double psi = current_pose[2];
    double xr = p[counter];
    double yr = q[counter];

    double s_prim = (c*exp(-alpha*ro)*vn)/hypot(p_prim[counter],q_prim[counter]);

    double xr_prim = p_prim[counter]*s_prim;
    double yr_prim = q_prim[counter]*s_prim;


    if(counter > 0) {
        xr_calc[counter] = xr_calc[counter-1] + Ts*xr_prim;
        yr_calc[counter] = yr_calc[counter-1] + Ts*yr_prim;
    }

    double delta_x = xr_calc[counter] - x;
    double delta_y = yr_calc[counter] - y;


    ro = hypot(delta_x,delta_y);
    ROS_DEBUG("ro: %f", ro);

    geometry_msgs::PoseStamped poza;
    poza.pose.position.x = xr_calc[counter];
    poza.pose.position.y = yr_calc[counter];
    vv_path.poses.push_back(poza);

    vv_path.header.frame_id = "map";
    vv_path_pub_.publish(vv_path);



    double psi_d = atan2(delta_y,delta_x);

    if(ro <= epsilon){
        double theta_r = atan2(yr,xr);
        psi_d = (psi_d*(-2*ro*ro*ro + 3*epsilon*ro*ro)+theta_r*(-2*(epsilon-ro)*(epsilon-ro)*(epsilon-ro)+3*((epsilon-ro)*(epsilon-ro))))/(epsilon*epsilon*epsilon);
    }

    double psi_d_prim = (psi_d - psi_d_prev)/Ts;

    psi_d_prev = psi_d;

    cmd_.speed = gama*ro*cos(psi_d_prev - psi);

    cmd_.rotation = k*(psi_d_prev - psi) + psi_d_prim;

    if (visualizer_->hasSubscriber()) {
        visualizer_->drawSteeringArrow(1, path_driver_->getRobotPoseMsg(), cmd_.direction_angle, 0.2, 1.0, 0.2);
    }


    ////////////////////////////////////////////////
    geometry_msgs::Point pt;
    pt.x = x;
    pt.y = y;
    robot_path.points.push_back(pt);

    points_pub_.publish(robot_path);
    /////////////////////////////////////////////////

    ROS_DEBUG("Error: %f", hypot(xr_calc[counter]-x,yr_calc[counter]-y));


    counter += 1;
    ROS_DEBUG("Counter: %d, N: %d", counter, N);
    if (counter >= N-1){
        counter = N-1;
        //if(ro <= 0.3){
            cmd_.speed = 0.0;
            cmd_.direction_angle = 0.0;
        //}

    }

    // NULL PTR
    setStatus(path_msgs::FollowPathResult::MOTION_STATUS_MOVING);


    publishCommand();
}

void RobotController_Omnidrive_VirtualVehicle::behaveAvoidObstacle()
{
    behaveOnLine();
}

bool RobotController_Omnidrive_VirtualVehicle::behaveApproachTurningPoint()
{
    behaveOnLine();
    return false;
}

void RobotController_Omnidrive_VirtualVehicle::reset()
{
    initialized = false;
}

