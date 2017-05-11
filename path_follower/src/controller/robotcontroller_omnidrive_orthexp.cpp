// THIRD PARTY
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

// PROJECT
#include <path_follower/parameters/path_follower_parameters.h>
#include <path_follower/controller/robotcontroller_omnidrive_orthexp.h>
#include <path_follower/utils/cubic_spline_interpolation.h>
#include <cslibs_utils/MathHelper.h>
#include <path_follower/utils/pose_tracker.h>
#include <path_follower/utils/visualizer.h>

// SYSTEM
#include <deque>
#include <Eigen/Dense>

#include <path_follower/factory/controller_factory.h>

REGISTER_ROBOT_CONTROLLER(RobotController_Omnidrive_OrthogonalExponential, omnidrive_orthexp, omnidrive);

using namespace Eigen;

namespace {
//! Module name, that is used for ros console output
const std::string MODULE = "controller";
}

RobotController_Omnidrive_OrthogonalExponential::RobotController_Omnidrive_OrthogonalExponential():
    RobotController(),
    cmd_(this),
    nh_("~"),
    view_direction_(LookInDrivingDirection),
    vn_(0.0),
    theta_des_(90.0*M_PI/180.0),
    Ts_(0.02),
    e_theta_curr_(0),
    curv_sum_(1e-3),
    distance_to_goal_(1e6),
    distance_to_obstacle_(1e3)
{
    look_at_cmd_sub_ = pnh_.subscribe<std_msgs::String>("/look_at/cmd", 10,
                                                       &RobotController_Omnidrive_OrthogonalExponential::lookAtCommand, this);
    look_at_sub_ = pnh_.subscribe<geometry_msgs::PointStamped>("/look_at", 10,
                                                              &RobotController_Omnidrive_OrthogonalExponential::lookAt, this);

    lookInDrivingDirection();

}

void RobotController_Omnidrive_OrthogonalExponential::stopMotion()
{
    cmd_.speed = 0;
    cmd_.direction_angle = 0;
    cmd_.rotation = 0;

    MoveCommand mcmd = cmd_;
    publishMoveCommand(mcmd);
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

void RobotController_Omnidrive_OrthogonalExponential::lookAt(const geometry_msgs::PointStampedConstPtr &look_at)
{
    look_at_ = look_at->point;
    view_direction_ = LookAtPoint;
}

void RobotController_Omnidrive_OrthogonalExponential::keepHeading()
{
    view_direction_ = KeepHeading;
    theta_des_ = pose_tracker_->getRobotPose()[2];
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
    RobotController::initialize();

    // initialize the desired angle and the angle error
    e_theta_curr_ = pose_tracker_->getRobotPose()[2];

    // desired velocity
    vn_ = std::min(PathFollowerParameters::getInstance()->max_velocity(), velocity_);
    ROS_WARN_STREAM_NAMED(MODULE, "velocity_: " << velocity_ << ", vn: " << vn_);
}


void RobotController_Omnidrive_OrthogonalExponential::start()
{

}

RobotController::MoveCommandStatus RobotController_Omnidrive_OrthogonalExponential::computeMoveCommand(MoveCommand *cmd)
{
    // omni drive can rotate.
    *cmd = MoveCommand(true);

    if(path_interpol.n() < 2) {
        ROS_ERROR("[Line] path is too short (N = %zu)", path_interpol.n());

        stopMotion();
        return MoveCommandStatus::REACHED_GOAL;
    }

    // get the pose as pose(0) = x, pose(1) = y, pose(2) = theta
    Eigen::Vector3d current_pose = pose_tracker_->getRobotPose();

    double x_meas = current_pose[0];
    double y_meas = current_pose[1];
    double theta_meas = current_pose[2];
    //***//

    RobotController::findOrthogonalProjection();
    double orth_proj = orth_proj_;

    if(RobotController::isGoalReached(cmd)){
       return RobotController::MoveCommandStatus::REACHED_GOAL;
    }

    //find the slope of the desired path, and plot a vector from the robot to the current point on the path

    double theta_p = path_interpol.theta_p(proj_ind_);

    visualization_msgs::Marker marker;
    marker.ns = "orthexp";
    marker.header.frame_id = getFixedFrame();
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
    to.x = path_interpol.p(proj_ind_);
    to.y = path_interpol.q(proj_ind_);

    marker.points.push_back(from);
    marker.points.push_back(to);

    visualizer_->getMarkerPublisher().publish(marker);

    //***//


    //check the "look-at" point, and calculate the rotation control

    switch(view_direction_) {
    case LookAtPoint:
        theta_des_ = std::atan2(look_at_.y - y_meas, look_at_.x - x_meas);
        break;
    case KeepHeading:
        // do nothing
        break;
    case LookInDrivingDirection:
        theta_des_ = cmd_.direction_angle + current_pose[2];
        break;
    case Rotate:
        theta_des_ += 0.01;
        break;
    default:
        throw std::runtime_error("unknown view direction mode");
        break;
    }

    double e_theta_new = MathHelper::NormalizeAngle(theta_des_ - theta_meas);
    double e_theta_prim = (e_theta_new - e_theta_curr_)/Ts_;

    e_theta_curr_ = e_theta_new;

    //***//

    //Calculate the look-ahead curvature

    //calculate the curvature, and stop when the look-ahead distance is reached (w.r.t. orthogonal projection)
    double s_cum_sum = 0;
    curv_sum_ = 1e-10;

    for (unsigned int i = proj_ind_ + 1; i < path_interpol.n(); i++){

        s_cum_sum = path_interpol.s(i) - path_interpol.s(proj_ind_);
        curv_sum_ += std::abs(path_interpol.curvature(i));

        if(s_cum_sum - opt_.look_ahead_dist() >= 0){
            break;
        }
    }

    double cum_sum_to_goal = 0;

    for(unsigned int i = proj_ind_ + 1; i < path_interpol.n(); i++){

        cum_sum_to_goal += hypot(path_interpol.p(i) - path_interpol.p(i-1), path_interpol.q(i) - path_interpol.q(i-1));

    }
    distance_to_goal_ = cum_sum_to_goal;

    double angular_vel = pose_tracker_->getVelocity().angular.z;
    //***//


    //control

    double exponent = opt_.k_curv()*curv_sum_
            + opt_.k_w()*fabs(angular_vel)
            + opt_.k_o()/distance_to_obstacle_
            + opt_.k_g()/distance_to_goal_;

    ROS_WARN_THROTTLE_NAMED(1, MODULE, "params:\ndistance to goal: %f\n"
                                       "distance to obstacles: %f\n"
                                       "angular vel.: %f\n"
                                       "curvature: %f",
                            distance_to_goal_, distance_to_obstacle_, angular_vel, curv_sum_);

    ROS_WARN_THROTTLE_NAMED(1, MODULE, "factor: %f\t%f\n"
                                       "distance to goal: %f\n"
                                       "distance to obstacles: %f\n"
                                       "angular vel.: %f\n"
                                       "curvature: %f",
                            exponent, std::exp(-exponent),
                            opt_.k_g()/distance_to_goal_, opt_.k_o()/distance_to_obstacle_, opt_.k_w()*fabs(angular_vel), opt_.k_curv()*fabs(curv_sum_));


    cmd_.speed = std::max(vn_*std::exp(-exponent),0.2);

    cmd_.direction_angle = atan(-opt_.k()*orth_proj) + theta_p - theta_meas;

    cmd_.rotation = opt_.kp()*e_theta_curr_ + opt_.kd()*e_theta_prim;

    if(cmd_.rotation > opt_.max_angular_velocity()) {
        cmd_.rotation = opt_.max_angular_velocity();
    } else if(cmd_.rotation < -opt_.max_angular_velocity()) {
        cmd_.rotation = -opt_.max_angular_velocity();
    }

    //***//


    if (visualizer_->hasSubscriber()) {
        visualizer_->drawSteeringArrow(pose_tracker_->getFixedFrameId(), 1, pose_tracker_->getRobotPoseMsg(), cmd_.direction_angle, 0.2, 1.0, 0.2);
    }


    //Vizualize the path driven by the robot
    geometry_msgs::Point pt;
    pt.x = x_meas;
    pt.y = y_meas;
    robot_path_marker_.points.push_back(pt);

    points_pub_.publish(robot_path_marker_);
    //***//


    // check for end
    double distance_to_goal = hypot(x_meas - path_interpol.p(path_interpol.n()-1), y_meas - path_interpol.q(path_interpol.n()-1));
    ROS_WARN_THROTTLE_NAMED(1, MODULE, "distance to goal: %f", distance_to_goal);


    *cmd = cmd_;

    return MoveCommandStatus::OKAY;

}

void RobotController_Omnidrive_OrthogonalExponential::publishMoveCommand(const MoveCommand &cmd) const
{
    geometry_msgs::Twist msg;

    Vector2f v = cmd.getVelocityVector();
    msg.linear.x  = v[0];
    msg.linear.y  = v[1];
    msg.angular.z = cmd.getRotationalVelocity();

    cmd_pub_.publish(msg);
}

