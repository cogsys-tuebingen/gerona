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

using namespace Eigen;


RobotController_Omnidrive_OrthogonalExponential::RobotController_Omnidrive_OrthogonalExponential(ros::Publisher &cmd_publisher,
                                                                                                 PathFollower *path_driver):
    RobotController(cmd_publisher, path_driver),
    cmd_(this),
    nh_("~"),
    has_look_at_(false),
    initialized(false),
    vn(0.0),
    Ts(0.02),
    N(0),
    e_theta_curr(0),
    theta_des(90.0*M_PI/180.0)


{
    visualizer_ = Visualizer::getInstance();
    interp_path_pub_ = nh_.advertise<nav_msgs::Path>("interp_path", 10);
    points_pub_ = nh_.advertise<visualization_msgs::Marker>("path_points", 10);

    look_at_sub_ = nh_.subscribe<geometry_msgs::PointStamped>("/look_at", 10,
                                                              boost::bind(&RobotController_Omnidrive_OrthogonalExponential::lookAt, this, _1));
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

void RobotController_Omnidrive_OrthogonalExponential::lookAt(const geometry_msgs::PointStampedConstPtr &look_at)
{
    look_at_ = look_at->point;
    has_look_at_ = true;
}

void RobotController_Omnidrive_OrthogonalExponential::setPath(PathWithPosition path)
{
    RobotController::setPath(path);

    if(initialized) {
        return;
    }


    initialized = true;

    std::vector<Waypoint> waypoints = *path_.current_path;

    // dirty hack!!!!!
    // TODO: find a way to do this correctly :-)
    waypoints.erase(waypoints.begin());
    waypoints.erase(waypoints.begin());
    waypoints.erase(waypoints.begin());
    waypoints.erase(waypoints.begin());

    //copy the waypoints to arrays X_arr and Y_arr, and introduce a new array l_arr_unif required for the interpolation
    N = waypoints.size();
    double X_arr[N], Y_arr[N], l_arr_unif[N];
    double f = std::max(0.0001, 1.0 / (double) (N-1));

    for(std::size_t i = 0; i < N; ++i) {
        const Waypoint& waypoint = waypoints[i];

        X_arr[i] = waypoint.x;
        Y_arr[i] = waypoint.y;
        l_arr_unif[i] = i * f;

    }
    //***//

    //desired velocity
    vn = path_driver_->getOptions().max_velocity_;
    // TODO: fix this mess
    if(vn > 1.0) {
        vn = 1.0;
    }
    //***//

    //initialization before the interpolation

    alglib::real_1d_array X_alg, Y_alg, l_alg_unif;

    X_alg.setcontent(N,X_arr);
    Y_alg.setcontent(N,Y_arr);
    l_alg_unif.setcontent(N,l_arr_unif);


    double x_s = 0.0, y_s = 0.0, x_s_prim = 0.0, y_s_prim = 0.0, x_s_sek = 0.0, y_s_sek = 0.0;


    alglib::spline1dinterpolant s_int1, s_int2;
    // FIXME: throws alglib::ap_error
    alglib::spline1dbuildcubic(l_alg_unif,X_alg,s_int1);
    alglib::spline1dbuildcubic(l_alg_unif,Y_alg,s_int2);

    p.clear();
    q.clear();
    p_prim.clear();
    q_prim.clear();
    interp_path.poses.clear();
    //***//

    //interpolate the path and find the derivatives, then publish the interpolated path
    for(uint i = 0; i < N; ++i) {
        geometry_msgs::PoseStamped poza;
        alglib::spline1ddiff(s_int1,l_alg_unif[i],x_s,x_s_prim,x_s_sek);
        alglib::spline1ddiff(s_int2,l_alg_unif[i],y_s,y_s_prim,y_s_sek);
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
    //***//

    //initialize the desired angle and the angle error
    double theta_meas = path_driver_->getRobotPose()[2];
    if(!has_look_at_) {
        theta_des = theta_meas;
    }
    e_theta_curr = theta_meas;
    //***//

    ////////////////////////////////////////////////
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


}

void RobotController_Omnidrive_OrthogonalExponential::initOnLine()
{

}

void RobotController_Omnidrive_OrthogonalExponential::behaveOnLine()
{

    //control parameters
    double k = 1.5, kp = 3.0, kd = 2.0;
    //***/

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
    if( ((theta_p > -M_PI/2) && (theta_p < M_PI/2) && (q[ind] > y_meas)) ||
            ((((theta_p > -M_PI) && (theta_p < -M_PI/2)) || ((theta_p > M_PI/2) && (theta_p < M_PI))) && (q[ind] < y_meas)) ){

        orth_proj *= -1;

    }
    ROS_DEBUG("Orthogonal distance: %f, theta_p: %f, theta_des: %f", orth_proj, theta_p*180.0/M_PI, theta_des*180.0/M_PI);

    //****//

    //////////////////////////////////////////////////////////////////

    //check the "look-at" point, and calculate the rotation control

    if(has_look_at_) {
        theta_des = std::atan2(look_at_.y - y_meas, look_at_.x - x_meas);
    }

    double e_theta_new = MathHelper::NormalizeAngle(theta_des - theta_meas);

//    if(std::abs(e_theta_new) > M_PI){
//        e_theta_new = MathHelper::NormalizeAngle(2*M_PI - e_theta_new);
//    }

    double e_theta_prim = (e_theta_new - e_theta_curr)/Ts;

    e_theta_curr = e_theta_new;

    //***//

    //control

    cmd_.speed = vn;
    cmd_.direction_angle = atan(-k*orth_proj) + theta_p - theta_meas;
    cmd_.rotation = kp*e_theta_curr + kd*e_theta_prim;

    //***//


    ROS_DEBUG("alpha: %f, alpha_e: %f, e_theta_curr: %f", (atan(-k*orth_proj) + theta_p)*180.0/M_PI, atan(-k*orth_proj)*180.0/M_PI, e_theta_curr);

    if (visualizer_->hasSubscriber()) {
        visualizer_->drawSteeringArrow(1, path_driver_->getRobotPoseMsg(), cmd_.direction_angle, 0.2, 1.0, 0.2);
    }


    ////////////////////////////////////////////////
    geometry_msgs::Point pt;
    pt.x = x_meas;
    pt.y = y_meas;
    robot_path.points.push_back(pt);

    points_pub_.publish(robot_path);

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
    behaveOnLine();

    Eigen::Vector3d current_pose = path_driver_->getRobotPose();
    double x_meas = current_pose[0];
    double y_meas = current_pose[1];

    double distance_to_goal = hypot(x_meas - p[N-1], y_meas - q[N-1]);
    ROS_WARN_THROTTLE(1, "distance to goal: %f", distance_to_goal);

    // TODO: waypoint tolerance from options
    return distance_to_goal <= 0.3;
}

void RobotController_Omnidrive_OrthogonalExponential::reset()
{
    initialized = false;
}

