// HEADER
#include <path_follower/legacy/robotcontroller_kinematic_SLP.h>

// THIRD PARTY
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

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
    vn_(0),
    delta_(0),
    Ts_(0.02),
    ind_(0),
    sign_v_(0),
    xe_(0),
    ye_(0),
    curv_sum_(1e-3),
    distance_to_goal_(1e-3),
    distance_to_obstacle_(1e-3)
{
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

    //reset the index of the current point on the path
    ind_ = 0;

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

    if(path_interpol.n() < 2) {
        ROS_ERROR("[Line] path is too short (N = %d)", (int) path_interpol.n());

        stopMotion();
        return MoveCommandStatus::REACHED_GOAL;
    }


    /// get the pose as pose(0) = x, pose(1) = y, pose(2) = theta
    Eigen::Vector3d current_pose = path_driver_->getRobotPose();

    double x_meas = current_pose[0];
    double y_meas = current_pose[1];
    double theta_meas = current_pose[2];
    ///***///


    ///calculate the control for the current point on the path

    //robot direction angle in path coordinates
    double theta = theta_meas - path_interpol.theta_p(ind_);

    //robot position vector module
    double r = hypot(x_meas - path_interpol.p(ind_), y_meas - path_interpol.q(ind_));

    //robot position vector angle in world coordinates
    double theta_r = atan2(y_meas - path_interpol.q(ind_), x_meas - path_interpol.p(ind_));

    //robot position vector angle in path coordinates
    double delta_theta = theta_r - path_interpol.theta_p(ind_);

    /////////////////////////////////////////////////////////////////////////////////////
    ROS_INFO("delta_theta not normalized: %lf", delta_theta*180.0/M_PI);
    /////////////////////////////////////////////////////////////////////////////////////

    int sign_xe = 1;
    int sign_ye = 1;

    //normalize delta_theta and determine the sign of path coordinates
    if((delta_theta > M_PI/2.0) & (delta_theta < M_PI))
    {
        delta_theta = M_PI - delta_theta;
        sign_xe = -1;
        sign_ye = 1;
    }
    
    else if((delta_theta > M_PI) & (delta_theta <= 3.0*M_PI/2.0))
    {
        delta_theta = delta_theta - M_PI;
        sign_xe = -1;
        sign_ye = -1;
    }

    else if((delta_theta > 3.0*M_PI/2.0) & (delta_theta <= 2.0*M_PI))
    {
        delta_theta = 2.0*M_PI - delta_theta;
        sign_xe = 1;
        sign_ye = -1;
    }

    else if((delta_theta < 0) & (delta_theta >= -M_PI/2.0))
    {
        delta_theta = std::abs(delta_theta);
        sign_xe = 1;
        sign_ye = -1;
    }
    
    else if((delta_theta < -M_PI/2.0) & (delta_theta >= -M_PI))
    {
        delta_theta = delta_theta + M_PI;
        sign_xe = -1;
        sign_ye = -1;
    }

    else if((delta_theta < -M_PI) & (delta_theta >= -3.0*M_PI/2.0))
    {
        delta_theta += M_PI;
        delta_theta = std::abs(delta_theta);
        sign_xe = -1;
        sign_ye = 1;
    }

    else if((delta_theta < -3.0*M_PI/2.0) & (delta_theta > -2.0*M_PI))
    {
        delta_theta = delta_theta + 2.0*M_PI;
        sign_xe = 1;
        sign_ye = 1;
    }

    /////////////////////////////////////////////////////////////////////////////////////
    ROS_INFO("delta_theta normalized: %lf", delta_theta*180.0/M_PI);
    /////////////////////////////////////////////////////////////////////////////////////
    
    
    //current robot position in path coordinates
    xe_ = sign_xe*r * cos(delta_theta);
    ye_ = sign_ye*r * sin(delta_theta);

    ///***///


    ///Calculate the parameters for exponential speed control

    //calculate the curvature, and stop when the look-ahead distance is reached (w.r.t. orthogonal projection)
    double s_cum_sum = 0;
    curv_sum_ = 1e-10;

    for (unsigned int i = ind_ + 1; i < path_interpol.n(); i++){

        s_cum_sum = path_interpol.s(i) - path_interpol.s(ind_);
        curv_sum_ += fabs(path_interpol.curvature(i));

        if(s_cum_sum - opt_.look_ahead_dist() >= 0){
            break;
        }
    }

    //calculate the distance from the orthogonal projection to the goal, w.r.t. path
    distance_to_goal_ = path_interpol.s(path_interpol.n()-1) - path_interpol.s(ind_);

    //get the robot's current angular velocity
    double angular_vel = path_driver_->getVelocity().angular.z;
    ///***///

    ///Calculate the delta_ and its derivative

    double delta_old = delta_;

    delta_ = -sign_v_*opt_.theta_a()*tanh(ye_);

    double delta_prim = (delta_ - delta_old)/Ts_;
    ///***///


    ///Control

    //ensure valid values
    if(distance_to_obstacle_ == 0 || !std::isfinite(distance_to_obstacle_)) distance_to_obstacle_ = 1e-10;
    if(distance_to_goal_ == 0 || !std::isfinite(distance_to_goal_)) distance_to_goal_ = 1e-10;

    double exponent = opt_.k_curv()*fabs(curv_sum_)
            + opt_.k_w()*fabs(angular_vel)
            + opt_.k_o()/distance_to_obstacle_
            + opt_.k_g()/distance_to_goal_;

    //TODO: consider the minimum excitation speed
    double v = std::max(0.2,vn_*exp(-exponent));
    cmd_.speed = v;

    cmd_.direction_angle = 0;

    cmd_.rotation = delta_prim - opt_.gamma()*ye_*v*(sin(theta) - sin(delta_))/(theta - delta_) - opt_.k2()*(theta - delta_);

    ///***///

    ///Get the velocity sign

    if(v > 0) sign_v_ = 1;
    else if (v < 0) sign_v_ = -1;
    else sign_v_ = 0;

    ///***///


    ///plot the robot position vector in path coordinates

    visualization_msgs::MarkerArray path_coord_marray;


    visualization_msgs::Marker path_robot_marker;
    path_robot_marker.ns = "path_robot_vector";
    path_robot_marker.header.frame_id = "/map";
    path_robot_marker.header.stamp = ros::Time();
    path_robot_marker.action = visualization_msgs::Marker::ADD;
    path_robot_marker.id = 0;
    path_robot_marker.color.r = 1;
    path_robot_marker.color.g = 1;
    path_robot_marker.color.b = 0;
    path_robot_marker.color.a = 1.0;
    path_robot_marker.scale.x = 0.05;
    path_robot_marker.scale.y = 0.02;
    path_robot_marker.scale.z = 0.02;
    path_robot_marker.type = visualization_msgs::Marker::ARROW;

    geometry_msgs::Point from, to;
    from.x = x_meas;
    from.y = y_meas;
    to.x = path_interpol.p(ind_);
    to.y = path_interpol.q(ind_);

    path_robot_marker.points.push_back(to);
    path_robot_marker.points.push_back(from);


    visualization_msgs::Marker path_abscissa_marker;
    path_abscissa_marker.ns = "path_coord_abscissa";
    path_abscissa_marker.header.frame_id = "/map";
    path_abscissa_marker.header.stamp = ros::Time();
    path_abscissa_marker.action = visualization_msgs::Marker::ADD;
    path_abscissa_marker.id = 0;
    path_abscissa_marker.color.r = 1;
    path_abscissa_marker.color.g = 0;
    path_abscissa_marker.color.b = 0;
    path_abscissa_marker.color.a = 1.0;
    path_abscissa_marker.scale.x = 0.3;
    path_abscissa_marker.scale.y = 0.05;
    path_abscissa_marker.scale.z = 0.05;
    path_abscissa_marker.type = visualization_msgs::Marker::ARROW;

    path_abscissa_marker.pose.position.x = path_interpol.p(ind_);
    path_abscissa_marker.pose.position.y = path_interpol.q(ind_);
    path_abscissa_marker.pose.position.z = 0.0;
    path_abscissa_marker.pose.orientation = tf::createQuaternionMsgFromYaw(path_interpol.theta_p(ind_));


    visualization_msgs::Marker abscissa_distance_marker;
    abscissa_distance_marker.ns = "abscissa_distance";
    abscissa_distance_marker.header.frame_id = "/map";
    abscissa_distance_marker.header.stamp = ros::Time();
    abscissa_distance_marker.action = visualization_msgs::Marker::ADD;
    abscissa_distance_marker.id = 0;
    abscissa_distance_marker.color.r = 0;
    abscissa_distance_marker.color.g = 1;
    abscissa_distance_marker.color.b = 1;
    abscissa_distance_marker.color.a = 1.0;
    abscissa_distance_marker.scale.x = xe_;
    abscissa_distance_marker.scale.y = 0.02;
    abscissa_distance_marker.scale.z = 0.02;
    abscissa_distance_marker.type = visualization_msgs::Marker::ARROW;

    abscissa_distance_marker.pose.position.x = path_interpol.p(ind_);
    abscissa_distance_marker.pose.position.y = path_interpol.q(ind_);
    abscissa_distance_marker.pose.position.z = 0.0;
    abscissa_distance_marker.pose.orientation = tf::createQuaternionMsgFromYaw(path_interpol.theta_p(ind_));


    visualization_msgs::Marker path_ordinate_marker;
    path_ordinate_marker.ns = "path_coord_ordinate";
    path_ordinate_marker.header.frame_id = "/map";
    path_ordinate_marker.header.stamp = ros::Time();
    path_ordinate_marker.action = visualization_msgs::Marker::ADD;
    path_ordinate_marker.id = 0;
    path_ordinate_marker.color.r = 0;
    path_ordinate_marker.color.g = 1;
    path_ordinate_marker.color.b = 0;
    path_ordinate_marker.color.a = 1.0;
    path_ordinate_marker.scale.x = 0.3;
    path_ordinate_marker.scale.y = 0.05;
    path_ordinate_marker.scale.z = 0.05;
    path_ordinate_marker.type = visualization_msgs::Marker::ARROW;

    path_ordinate_marker.pose.position.x = path_interpol.p(ind_);
    path_ordinate_marker.pose.position.y = path_interpol.q(ind_);
    path_ordinate_marker.pose.position.z = 0.0;
    path_ordinate_marker.pose.orientation = tf::createQuaternionMsgFromYaw(path_interpol.theta_p(ind_) + M_PI/2.0);


    visualization_msgs::Marker ordinate_distance_marker;
    ordinate_distance_marker.ns = "ordinate_distance";
    ordinate_distance_marker.header.frame_id = "/map";
    ordinate_distance_marker.header.stamp = ros::Time();
    ordinate_distance_marker.action = visualization_msgs::Marker::ADD;
    ordinate_distance_marker.id = 0;
    ordinate_distance_marker.color.r = 0;
    ordinate_distance_marker.color.g = 1;
    ordinate_distance_marker.color.b = 1;
    ordinate_distance_marker.color.a = 1.0;
    ordinate_distance_marker.scale.x = ye_;
    ordinate_distance_marker.scale.y = 0.02;
    ordinate_distance_marker.scale.z = 0.02;
    ordinate_distance_marker.type = visualization_msgs::Marker::ARROW;

    ordinate_distance_marker.pose.position.x = path_interpol.p(ind_);
    ordinate_distance_marker.pose.position.y = path_interpol.q(ind_);
    ordinate_distance_marker.pose.position.z = 0.0;
    ordinate_distance_marker.pose.orientation = tf::createQuaternionMsgFromYaw(path_interpol.theta_p(ind_) + M_PI/2.0);



    path_coord_marray.markers.push_back(visualization_msgs::Marker(path_robot_marker));
    path_coord_marray.markers.push_back(visualization_msgs::Marker(path_abscissa_marker));
    path_coord_marray.markers.push_back(visualization_msgs::Marker(abscissa_distance_marker));
    path_coord_marray.markers.push_back(visualization_msgs::Marker(path_ordinate_marker));
    path_coord_marray.markers.push_back(visualization_msgs::Marker(ordinate_distance_marker));

    visualizer_->getMarkerArrayPublisher().publish(path_coord_marray);

    ///***///

    ///////////////////////////////////////////////////////////////
    ROS_INFO("Index: %d", ind_);
    ROS_INFO("Theta_r: %lf", theta_r*180.0/M_PI);
    ROS_INFO("Theta_p: %lf", path_interpol.theta_p(ind_)*180.0/M_PI);
    ROS_INFO("Delta_theta: %lf", delta_theta*180.0/M_PI);
    ROS_INFO("Theta_meas: %lf", theta_meas*180.0/M_PI);
    ROS_INFO("Theta: %lf", theta*180.0/M_PI);
    ROS_INFO("Speed: %lf", v);
    ROS_INFO("r: %lf", r);
    ROS_INFO("xe: %lf, ye: %lf", xe_, ye_);
    ROS_INFO("s_new: %lf", path_interpol.s_new());
    ///////////////////////////////////////////////////////////////

    ///Calculate the next point on the path

    double s_old = path_interpol.s_new();

    //calculate the speed of the "virtual vehicle"
    path_interpol.set_s_prim(v * cos(theta) + opt_.k1() * xe_);

    //approximate the first derivative and calculate the next point
    double s_temp = Ts_*path_interpol.s_prim() + s_old;
    path_interpol.set_s_new(s_temp > 0 ? s_temp : 0);

    ///***///


    ///Calculate the index of the new point

    double s_diff = std::numeric_limits<double>::max();
    uint old_ind = ind_;

    for (unsigned int i = old_ind; i < path_interpol.n(); i++){

        double s_diff_curr = std::abs(path_interpol.s_new() - path_interpol.s(i));

        if(s_diff_curr < s_diff){

            s_diff = s_diff_curr;
            ind_ = i;

        }

    }

    ///////////////////////////////////////////////////////////////
    ROS_INFO("Index: %d", ind_);
    ROS_INFO("s_new: %lf", path_interpol.s_new());
    ROS_INFO("s_prim: %lf", path_interpol.s_prim());
    ///////////////////////////////////////////////////////////////

    ///***///


    if (visualizer_->hasSubscriber()) {
        visualizer_->drawSteeringArrow(1, path_driver_->getRobotPoseMsg(), cmd_.direction_angle, 0.2, 1.0, 0.2);
    }

    //***//


    // check for end
    double distance_to_goal_eucl = hypot(x_meas - path_interpol.p(path_interpol.n()-1), y_meas - path_interpol.q(path_interpol.n()-1));
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
