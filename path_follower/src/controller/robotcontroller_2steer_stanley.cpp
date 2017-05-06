#include <path_follower/controller/robotcontroller_2steer_stanley.h>



#include <path_follower/utils/pose_tracker.h>
#include <ros/ros.h>

#include <cslibs_utils/MathHelper.h>

#include <visualization_msgs/Marker.h>
#include <path_follower/utils/visualizer.h>

#include <deque>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <limits>
#include <boost/algorithm/clamp.hpp>


#ifdef TEST_OUTPUT
#include <std_msgs/Float64MultiArray.h>
#endif

#include <path_follower/factory/controller_factory.h>

REGISTER_ROBOT_CONTROLLER(RobotController_2Steer_Stanley, 2steer_stanley, default_collision_avoider);

using namespace std;
using namespace Eigen;

RobotController_2Steer_Stanley::RobotController_2Steer_Stanley():
    RobotController()
{

    ROS_INFO("Parameters: k_forward=%f, k_backward=%f\n"
                "vehicle_length=%f\n"
                "goal_tolerance=%f",
                params_.k_forward(), params_.k_backward(),
                params_.vehicle_length(),
                params_.goal_tolerance());

#ifdef TEST_OUTPUT
    test_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("test_output", 100);
#endif

}

void RobotController_2Steer_Stanley::stopMotion() {

    move_cmd_.setVelocity(0.f);
    move_cmd_.setDirection(0.f);

    MoveCommand cmd = move_cmd_;
    publishMoveCommand(cmd);
}

void RobotController_2Steer_Stanley::start() {

}

void RobotController_2Steer_Stanley::setPath(Path::Ptr path) {
    RobotController::setPath(path);
}

RobotController::MoveCommandStatus RobotController_2Steer_Stanley::computeMoveCommand(
        MoveCommand* cmd) {

    if(path_interpol.n() <= 2)
        return RobotController::MoveCommandStatus::ERROR;

    const Eigen::Vector3d pose = pose_tracker_->getRobotPose();
    const geometry_msgs::Twist velocity_measured = pose_tracker_->getVelocity();

    RobotController::findOrthogonalProjection();
    double d = -orth_proj_;

    if(RobotController::isGoalReached(cmd)){
       return RobotController::MoveCommandStatus::REACHED_GOAL;
    }

    // draw a line to the orthogonal projection
    geometry_msgs::Point from, to;
    from.x = pose[0]; from.y = pose[1];
    to.x = path_interpol.p(proj_ind_); to.y = path_interpol.q(proj_ind_);
    visualizer_->drawLine(12341234, from, to, getFixedFrame(), "kinematic", 1, 0, 0, 1, 0.01);


    // theta_e = theta_vehicle - theta_path (orientation error)
    double theta_e = MathHelper::AngleDelta(pose[2], path_interpol.theta_p(proj_ind_));

    // if we drive backwards invert d and set theta_e to the complementary angle
    if (getDirSign() < 0.)
        theta_e = theta_e > 0.? -M_PI + theta_e : M_PI + theta_e;

    const double k = getDirSign() > 0. ? params_.k_forward() : params_.k_backward();
    const double v = max(abs(velocity_measured.linear.x), 0.3);

    // steering angle
    double phi = theta_e + atan2(k * d, v);

    if (phi == NAN)
        phi = 0.;

    phi = boost::algorithm::clamp(phi, -params_.max_steering_angle(), params_.max_steering_angle());

    move_cmd_.setDirection(getDirSign() * (float) phi);
    move_cmd_.setVelocity(getDirSign() * (float) velocity_);

#ifdef TEST_OUTPUT
    publishTestOutput(proj_ind_, d, theta_e, phi, v);
#endif

    *cmd = move_cmd_;

    return RobotController::MoveCommandStatus::OKAY;
}

void RobotController_2Steer_Stanley::publishMoveCommand(
        const MoveCommand& cmd) const {

    geometry_msgs::Twist msg;
    msg.linear.x  = cmd.getVelocity();
    msg.linear.y  = 0;
    msg.angular.z = cmd.getDirectionAngle();

    cmd_pub_.publish(msg);
}

#ifdef TEST_OUTPUT
void RobotController_2Steer_Stanley::publishTestOutput(const unsigned int waypoint, const double d,
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
