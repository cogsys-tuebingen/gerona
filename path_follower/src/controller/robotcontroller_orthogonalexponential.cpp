// HEADER
#include <path_follower/controller/robotcontroller_orthogonalexponential.h>

// PROJECT
#include <path_follower/parameters/path_follower_parameters.h>
#include <path_follower/utils/cubic_spline_interpolation.h>
#include <cslibs_navigation_utilities/MathHelper.h>
#include <path_follower/utils/pose_tracker.h>
#include <path_follower/utils/visualizer.h>

// SYSTEM
#include <deque>

using namespace Eigen;


RobotController_OrthogonalExponential::RobotController_OrthogonalExponential():
    RobotController(),
    cmd_(this),
    nh_("~"),
    view_direction_(LookInDrivingDirection),
    vn_(0.0),
    theta_des_(90.0*M_PI/180.0),
    Ts_(0.02),
    alpha_e_(0.0)
{
    look_at_cmd_sub_ = nh_.subscribe<std_msgs::String>("look_at/cmd", 10,
                                                       &RobotController_OrthogonalExponential::lookAtCommand, this);
    look_at_sub_ = nh_.subscribe<geometry_msgs::PointStamped>("look_at", 10,
                                                              &RobotController_OrthogonalExponential::lookAt, this);
    lookInDrivingDirection();
}

void RobotController_OrthogonalExponential::stopMotion()
{
    cmd_.speed = 0;
    cmd_.direction_angle = 0;
    cmd_.rotation = 0;

    MoveCommand mcmd = cmd_;
    publishMoveCommand(mcmd);
}

void RobotController_OrthogonalExponential::lookAtCommand(const std_msgs::StringConstPtr &cmd)
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

void RobotController_OrthogonalExponential::lookAt(const geometry_msgs::PointStampedConstPtr &look_at)
{
    look_at_ = look_at->point;
    view_direction_ = LookAtPoint;
}

void RobotController_OrthogonalExponential::keepHeading()
{
    view_direction_ = KeepHeading;
    theta_des_ = pose_tracker_->getRobotPose()[2];
}

void RobotController_OrthogonalExponential::rotate()
{
    view_direction_ = Rotate;
}

void RobotController_OrthogonalExponential::lookInDrivingDirection()
{
    view_direction_ = LookInDrivingDirection;
}

void RobotController_OrthogonalExponential::initialize()
{
    RobotController::initialize();

    // desired velocity
    vn_ = std::min(PathFollowerParameters::getInstance()->max_velocity(), velocity_);
    ROS_DEBUG_STREAM("velocity_: " << velocity_ << ", vn: " << vn_);
}

void RobotController_OrthogonalExponential::start()
{

}

void RobotController_OrthogonalExponential::computeControl()
{

}

RobotController::MoveCommandStatus RobotController_OrthogonalExponential::computeMoveCommand(MoveCommand *cmd)
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
    //***//

    RobotController::findOrthogonalProjection();

    if(RobotController::isGoalReached(cmd)){
       return RobotController::MoveCommandStatus::REACHED_GOAL;
    }

    //find the slope of the desired path, and plot a vector from the robot to the current point on the path

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

    //***//

    //compute the control commands
    computeControl();


    if (visualizer_->hasSubscriber()) {
        visualizer_->drawSteeringArrow(pose_tracker_->getFixedFrameId(), 1, pose_tracker_->getRobotPoseMsg(), cmd_.direction_angle, 0.2, 1.0, 0.2);
    }

    *cmd = cmd_;

    return MoveCommandStatus::OKAY;

}

void RobotController_OrthogonalExponential::publishMoveCommand(const MoveCommand &cmd) const
{
    geometry_msgs::Twist msg;
    if(cmd.getDirectionAngle() == 0){
        msg.linear.x  = cmd.getVelocity();
        msg.linear.y  = 0;

    }
    else{
        Vector2f v = cmd.getVelocityVector();
            msg.linear.x  = v[0];
            msg.linear.y  = v[1];
    }

    msg.angular.z = cmd.getRotationalVelocity();


    cmd_pub_.publish(msg);
}


