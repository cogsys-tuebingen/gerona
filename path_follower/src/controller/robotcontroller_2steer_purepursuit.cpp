/*
 *
 *  Created on: Apr 25, 2015
 *      Author: Lukas Hollaender
 */

#include <path_follower/controller/robotcontroller_2steer_purepursuit.h>


#include <ros/ros.h>
#include <path_follower/utils/pose_tracker.h>
#include <path_follower/utils/visualizer.h>

#include <cslibs_utils/MathHelper.h>

#include <visualization_msgs/Marker.h>

#include <deque>
#include <Eigen/Core>
#include <Eigen/Dense>

#ifdef TEST_OUTPUT
#include <std_msgs/Float64MultiArray.h>
#endif

#include <path_follower/factory/controller_factory.h>

REGISTER_ROBOT_CONTROLLER(RobotController_2Steer_PurePursuit, 2steer_purepursuit);

using namespace std;
using namespace Eigen;

RobotController_2Steer_PurePursuit::RobotController_2Steer_PurePursuit ():
    RobotController(),
    waypoint_(0)
{

    ROS_INFO("Parameters: k_forward=%f, k_backward=%f"
             "\nvehicle_length=%f\ngoal_tolerance=%f",
             params_.k_forward(), params_.k_backward(),
             params_.vehicle_length(), params_.goal_tolerance());

#ifdef TEST_OUTPUT
    test_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/test_output", 100);
#endif
}

void RobotController_2Steer_PurePursuit::reset() {
    waypoint_ = 0;
    RobotController::reset();
}

void RobotController_2Steer_PurePursuit::setPath(Path::Ptr path) {
    RobotController::setPath(path);
}


void RobotController_2Steer_PurePursuit::stopMotion() {

    move_cmd_.setVelocity(0.f);
    move_cmd_.setDirection(0.f);

    MoveCommand cmd = move_cmd_;
    publishMoveCommand(cmd);
}

void RobotController_2Steer_PurePursuit::start() {

}

RobotController::MoveCommandStatus RobotController_2Steer_PurePursuit::computeMoveCommand(
        MoveCommand* cmd) {

    if(path_interpol.n() <= 2)
        return RobotController::MoveCommandStatus::ERROR;

    const Eigen::Vector3d pose = pose_tracker_->getRobotPose();
    const geometry_msgs::Twist velocity_measured = pose_tracker_->getVelocity();

    RobotController::findOrthogonalProjection();
    double d = orth_proj_;

    if(RobotController::isGoalReached(cmd)){
       return RobotController::MoveCommandStatus::REACHED_GOAL;
    }

    const double v = velocity_measured.linear.x;
    double l_ah = v * (getDirSign() > 0.? params_.k_forward() : params_.k_backward());
    l_ah = max(l_ah, 0.4);

    // angle between vehicle theta and the connection between the reference point and the look ahead point
    const double alpha = computeAlpha(l_ah, pose);

    const double phi = atan2(params_.vehicle_length() * sin(alpha), l_ah);

    if (phi == NAN) {
        ROS_ERROR("Got NAN phi");
        return RobotController::MoveCommandStatus::ERROR;
    }

    move_cmd_.setDirection(getDirSign() * (float) phi);
    move_cmd_.setVelocity(getDirSign() * (float) velocity_);

    *cmd = move_cmd_;

#ifdef TEST_OUTPUT
    double theta_e = MathHelper::AngleDelta(pose[2], path_interpol.theta_p(proj_ind_));

    if (getDirSign() < 0.) {
        d = -d;
        theta_e = theta_e > 0.? M_PI - theta_e : -M_PI - theta_e;
    }

    publishTestOutput(proj_ind_, d, theta_e, phi, v);
#endif

    return RobotController::MoveCommandStatus::OKAY;
}

void RobotController_2Steer_PurePursuit::publishMoveCommand(
        const MoveCommand& cmd) const {

    geometry_msgs::Twist msg;
    msg.linear.x  = cmd.getVelocity();
    msg.linear.y  = 0;
    msg.angular.z = cmd.getDirectionAngle();

    cmd_pub_.publish(msg);
}

double RobotController_2Steer_PurePursuit::computeAlpha(double& l_ah, const Eigen::Vector3d& pose) {

    double distance, dx, dy;
    for (unsigned int i = waypoint_; i < path_interpol.n(); ++i) {
        dx = path_interpol.p(i) - pose[0];
        dy = path_interpol.q(i) - pose[1];

        distance = hypot(dx, dy);
        waypoint_ = i;
        if (distance >= l_ah)
            break;
    }

    // angle between the connection line and the vehicle orientation
    double alpha = MathHelper::AngleDelta(pose[2], atan2(dy, dx));

    // when we drive backwards, set alpha to the complementary angle
    if (getDirSign() < 0.)
        alpha = alpha > 0.? -M_PI + alpha : M_PI + alpha;

    // set lookahead_distance to the actual distance
    l_ah = distance;

    // line to lookahead point
    geometry_msgs::Point from, to;
    from.x = pose[0]; from.y = pose[1];
    to.x = path_interpol.p(waypoint_); to.y = path_interpol.q(waypoint_);
    visualizer_->drawLine(12341234, from, to, getFixedFrame(), "geo", 1, 0, 0, 1, 0.01);

    return alpha;
}

#ifdef TEST_OUTPUT
void RobotController_2Steer_PurePursuit::publishTestOutput(const unsigned int waypoint, const double d,
                                                           const double theta_e,
                                                           const double phi, const double v) const {
    std_msgs::Float64MultiArray msg;

    msg.data.push_back((double) waypoint);
    msg.data.push_back(d);
    msg.data.push_back(theta_e);
    msg.data.push_back(phi);
    msg.data.push_back(v);

    test_pub_.publish(msg);
}
#endif
