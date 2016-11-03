// HEADER
#include <path_follower/controller/robotcontroller_differential_orthexp.h>

// THIRD PARTY
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

// PROJECT
#include <path_follower/pathfollower.h>
#include <path_follower/utils/cubic_spline_interpolation.h>
#include <interpolation.h>
#include <cslibs_utils/MathHelper.h>

// SYSTEM
#include <deque>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;


RobotController_Differential_OrthogonalExponential::RobotController_Differential_OrthogonalExponential(PathFollower *path_driver):
    RobotController_Interpolation(path_driver),
    cmd_(this),
    nh_("~"),
    view_direction_(LookInDrivingDirection),
    vn_(0.0),
    theta_des_(90.0*M_PI/180.0),
    Ts_(0.02),
    alpha_e_(0.0),
    curv_sum_(1e-3),
    distance_to_goal_(1e6),
    distance_to_obstacle_(1e3)
{
    look_at_cmd_sub_ = nh_.subscribe<std_msgs::String>("/look_at/cmd", 10,
                                                       &RobotController_Differential_OrthogonalExponential::lookAtCommand, this);
    look_at_sub_ = nh_.subscribe<geometry_msgs::PointStamped>("/look_at", 10,
                                                              &RobotController_Differential_OrthogonalExponential::lookAt, this);
    lookInDrivingDirection();
}

void RobotController_Differential_OrthogonalExponential::stopMotion()
{
    cmd_.speed = 0;
    cmd_.direction_angle = 0;

    MoveCommand mcmd = cmd_;
    publishMoveCommand(mcmd);
}

void RobotController_Differential_OrthogonalExponential::lookAtCommand(const std_msgs::StringConstPtr &cmd)
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

void RobotController_Differential_OrthogonalExponential::lookAt(const geometry_msgs::PointStampedConstPtr &look_at)
{
    look_at_ = look_at->point;
    view_direction_ = LookAtPoint;
}

void RobotController_Differential_OrthogonalExponential::keepHeading()
{
    view_direction_ = KeepHeading;
    theta_des_ = path_driver_->getRobotPose()[2];
}

void RobotController_Differential_OrthogonalExponential::rotate()
{
    view_direction_ = Rotate;
}

void RobotController_Differential_OrthogonalExponential::lookInDrivingDirection()
{
    view_direction_ = LookInDrivingDirection;
}

void RobotController_Differential_OrthogonalExponential::initialize()
{
    RobotController_Interpolation::initialize();

    // desired velocity
    vn_ = std::min(path_driver_->getOptions().max_velocity(), velocity_);
    ROS_WARN_STREAM("velocity_: " << velocity_ << ", vn: " << vn_);
}

void RobotController_Differential_OrthogonalExponential::start()
{
    path_driver_->getCoursePredictor().reset();
}



RobotController::MoveCommandStatus RobotController_Differential_OrthogonalExponential::computeMoveCommand(MoveCommand *cmd)
{
    // omni drive can rotate.
    *cmd = MoveCommand(true);

    if(path_interpol.n() < 2) {
        ROS_ERROR("[Line] path is too short (N = %u)", path_interpol.n());

        stopMotion();
        return MoveCommandStatus::REACHED_GOAL;
    }

//    Vector2d dir_of_mov = path_driver_->getCoursePredictor().smoothedDirection();
//    if (!dir_of_mov.isZero() && path_driver_->isObstacleAhead(MathHelper::Angle(dir_of_mov))) {
//        ROS_WARpath_interpol.n()THROTTLE(1, "Collision!");
//        //TODO: not so good to use result-constant if it is not finishing the action...
//        setStatus(path_msgs::FollowPathResult::MOTIOpath_interpol.n()STATUS_OBSTACLE);

//        stopMotion();
//        path_driver_->getCoursePredictor().freeze();

//        return OBSTACLE;
//    }
//    path_driver_->getCoursePredictor().unfreeze();

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

    for (unsigned int i = 0; i < path_interpol.n(); i++){

        dist = hypot(x_meas - path_interpol.p(i), y_meas - path_interpol.q(i));
        if(dist < orth_proj){

            orth_proj = dist;
            ind = i;

        }

    }
    //***//

    //find the slope of the desired path, and plot a vector from the robot to the current point on the path

    double theta_p = path_interpol.theta_p(ind);

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
    to.x = path_interpol.p(ind);
    to.y = path_interpol.q(ind);

    marker.points.push_back(from);
    marker.points.push_back(to);

    visualizer_->getMarkerPublisher().publish(marker);

    //***//

    //determine the sign of the orthogonal distance
    static const double epsilon = 1e-3;
    if( ((theta_p > -M_PI/2) && (theta_p < M_PI/2) && (path_interpol.q(ind) > y_meas))
            || ((((theta_p >= -M_PI) && (theta_p < -M_PI/2)) || ((theta_p > M_PI/2) && (theta_p <= M_PI)))
            && (path_interpol.q(ind) < y_meas))
            || ((std::abs(theta_p + M_PI/2) < epsilon) && (path_interpol.p(ind) > x_meas))
            || ((std::abs(theta_p - M_PI/2) < epsilon) && (path_interpol.p(ind) < x_meas)) ){

        orth_proj = -fabs(orth_proj);

    }else{
        orth_proj = fabs(orth_proj);
    }

    //****//


    //check the "look-at" point, and calculate the rotation control

    switch(view_direction_) {
    case LookAtPoint:
        theta_des_ = std::atan2(look_at_.y - y_meas, look_at_.x - x_meas);
        break;
    case KeepHeading:
        // do nothing
        break;
    case LookInDrivingDirection:
        theta_des_ = cmd_.direction_angle + current_pose[2];//std::atan2(q[ind+1] - y_meas, p[ind+1] - x_meas);
        break;
    case Rotate:
        theta_des_ += 0.01;
        break;
    default:
        throw std::runtime_error("unknown view direction mode");
        break;
    }

    //***//

    //Calculate the look-ahead curvature

    /*uint look_ahead_index;
    double look_ahead_difference = std::numeric_limits<double>::max();*/

    double look_ahead_cum_sum = 0;
    curv_sum_ = 1e-10;


    for (unsigned int i = ind + 1; i < path_interpol.n(); i++){

        look_ahead_cum_sum += hypot(path_interpol.p(i) - path_interpol.p(i-1), path_interpol.q(i) - path_interpol.q(i-1));
        curv_sum_ += fabs(path_interpol.curvature(i));

        if(look_ahead_cum_sum - opt_.look_ahead_dist() >= 0){
            break;
        }
    }


    double cum_sum_to_goal = 0;

    for(unsigned int i = ind + 1; i < path_interpol.n(); i++){

        cum_sum_to_goal += hypot(path_interpol.p(i) - path_interpol.p(i-1), path_interpol.q(i) - path_interpol.q(i-1));

    }
    distance_to_goal_ = cum_sum_to_goal;

    //distance_to_goal_ = hypot(x_meas - path_interpol.p(path_interpol.n()-1), y_meas - path_interpol.q(path_interpol.n()-1));

    double angular_vel = path_driver_->getVelocity().angular.z;
    //***//


    //control

    double exponent = opt_.k_curv()*fabs(curv_sum_)
            + opt_.k_w()*fabs(angular_vel)
            + opt_.k_o()/distance_to_obstacle_
            + opt_.k_g()/distance_to_goal_;

    cmd_.speed = std::max(vn_*exp(-exponent),0.2);

    double last_alpha_e = alpha_e_;
    alpha_e_ = atan(-opt_.k()*orth_proj);
    cmd_.direction_angle = alpha_e_ + theta_p - theta_meas;

    cmd_.rotation = (alpha_e_ - last_alpha_e) / Ts_ + cmd_.direction_angle;

    //***//

    if (visualizer_->hasSubscriber()) {
        visualizer_->drawSteeringArrow(path_driver_->getFixedFrameId(), 1, path_driver_->getRobotPoseMsg(), cmd_.direction_angle, 0.2, 1.0, 0.2);
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
    ROS_WARN_THROTTLE(1, "distance to goal: %f", distance_to_goal);


    if(distance_to_goal <= path_driver_->getOptions().goal_tolerance()) {
        return MoveCommandStatus::REACHED_GOAL;
    } else {
        *cmd = cmd_;

        return MoveCommandStatus::OKAY;
    }
}

void RobotController_Differential_OrthogonalExponential::publishMoveCommand(const MoveCommand &cmd) const
{
    geometry_msgs::Twist msg;
    msg.linear.x  = cmd.getVelocity();
    msg.linear.y  = 0;
    msg.angular.z = cmd.getDirectionAngle();

    cmd_pub_.publish(msg);
}


