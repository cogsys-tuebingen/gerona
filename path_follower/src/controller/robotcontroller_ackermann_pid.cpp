#include <path_follower/controller/robotcontroller_ackermann_pid.h>
#include <cmath>
#include <path_msgs/FollowPathResult.h>
#include <path_follower/pathfollower.h>
#include <path_follower/utils/path_exceptions.h>

using namespace std;

namespace {
//! Module name, that is used for ros console output
const std::string MODULE = "controller";

/**
 * \brief Signum function
 * \author user79758@stackoverflow
 *
 * \see http://stackoverflow.com/a/4609795/2095383
 */
template <typename T> int sign(T val) {
    return (T(0) < val) - (val < T(0));
}
}

RobotController_Ackermann_Pid::RobotController_Ackermann_Pid(PathFollower *path_driver):
    RobotController(path_driver),
    behaviour_(ON_PATH),
    last_velocity_(0)
{
    // initialize
    steer_pid_ = PidController<1>(opt_.pid_kp(), opt_.pid_ki(), opt_.pid_kd(), opt_.pid_ta());
}

void RobotController_Ackermann_Pid::stopMotion()
{
    cmd_.setVelocity(0);
    publishMoveCommand(cmd_);
}

void RobotController_Ackermann_Pid::reset()
{
    behaviour_ = ON_PATH;
}

RobotController::MoveCommandStatus RobotController_Ackermann_Pid::computeMoveCommand(MoveCommand *cmd)
{
    /* This is a reimplemented, simplified version of the old behaviour based Ackermann
     * controller. There is still a internal state called "behaviour", but this is not a strict
     * state machine and there are no behaviour classes anymore.
     *
     * The path is processed as follows:
     *  - At every time, only the current subpath is taken into account.
     *  - While driving on a subpath, the controller tries to minimize the distance to the line
     *    of the current path segment (defined by the next two waypoints). behaviour_ = ON_PATH.
     *  - When reaching the last waypoint of the current sub path, the behaviour switches to
     *    APPROACH_SUBPATH_END and tries to hit the waypoint as close as possible.
     *  - When this last waypoint is reached, the robot is stopped and the controller waits,
     *    until there is no more movement (behaviour_ = WAIT_FOR_STOP)
     *  - When the robot really has stopped, the whole procedure starts again with the next sub
     *    path, until the end of the last sub path (= the goal) is reached
     */


    // If wait_for_stop_ is set, do nothing, until the actual velocity fell below a given
    // threshold.
    // When the robot has finally stopped, go to the next subpath or quit, if goal is reached.
    if (behaviour_ == WAIT_FOR_STOP) {
        stopMotion(); //< probably not necessary to repeat this, but be on the save side.

        // do nothing until robot has realy stopped.
        geometry_msgs::Twist current_vel = path_driver_->getVelocity();
        if((std::abs(current_vel.linear.x) > 0.01) ||
           (std::abs(current_vel.linear.y) > 0.01) ||
           (std::abs(current_vel.angular.z) > 0.01)) {
            ROS_INFO_THROTTLE_NAMED(1, MODULE, "WAITING until no more motion");
            return MoveCommandStatus::OKAY;
        } else {
            ROS_INFO_NAMED(MODULE, "Done at waypoint -> reset");
            path_->switchToNextSubPath();
            if(path_->isDone()) {
                return MoveCommandStatus::REACHED_GOAL;
            }
            behaviour_ = ON_PATH; // not necessary to set this explicitly, but it is more clear.
        }
    }


    // choose waypoint for this step
    selectWaypoint();

    // check if done (if last step was ATP and the direction sign flipped)
    float dir_sign = sign<double>(next_wp_local_.x());
    if(behaviour_ == APPROACH_SUBPATH_END && dir_sign != getDirSign()) {
        stopMotion();
        behaviour_ = WAIT_FOR_STOP;
        return MoveCommandStatus::OKAY;
    }
    setDirSign(dir_sign);


    float error;
    if (path_->isLastWaypoint()) {
        behaviour_ = APPROACH_SUBPATH_END;
        error = getErrorApproachSubpathEnd();
    } else {
        behaviour_ = ON_PATH;
        error = getErrorOnPath();
    }

    updateCommand(error);
    *cmd = cmd_;

    return MoveCommandStatus::OKAY;
}

void RobotController_Ackermann_Pid::publishMoveCommand(const MoveCommand &cmd) const
{
    geometry_msgs::Twist msg;
    if (cmd.isValid()) {
        msg.linear.x = cmd.getVelocity();
        msg.angular.z = cmd.getDirectionAngle();
        last_velocity_ = cmd.getVelocity();
    }
    // else (=not valid): dont modify msg --> all set to zero, that is robot will stop.
    cmd_pub_.publish(msg);
}

void RobotController_Ackermann_Pid::selectWaypoint()
{
    double tolerance = path_driver_->getOptions().wp_tolerance();

    // increase tolerance, when driving backwards
    if(getDirSign() < 0) {
        tolerance *= 2;
    }

    // switch to the nearest waypoint, that is at least 'tolerance' far away.
    while (!path_->isLastWaypoint() &&
           distanceToWaypoint(path_->getCurrentWaypoint()) < tolerance) {
        path_->switchToNextWaypoint();
    }

    if (visualizer_->hasSubscriber()) {
        visualizer_->drawArrow(path_driver_->getFixedFrameId(), 0, path_->getCurrentWaypoint(), "current waypoint", 1, 1, 0);
        visualizer_->drawArrow(path_driver_->getFixedFrameId(), 1, path_->getLastWaypoint(), "current waypoint", 1, 0, 0);
    }

    // convert waypoint to local frame. NOTE: This has to be done, even if the waypoint did not
    // change, as its position in the local frame changes while the robot moves.
    geometry_msgs::PoseStamped wp_map;
    wp_map.pose = path_->getCurrentWaypoint();
    wp_map.header.stamp = ros::Time::now();

    if (!path_driver_->transformToLocal(wp_map, next_wp_local_)) {
        throw EmergencyBreakException("cannot transform next waypoint",
                                      path_msgs::FollowPathResult::RESULT_STATUS_TF_FAIL);
    }
}

float RobotController_Ackermann_Pid::getErrorOnPath()
{
    /* The error is the sum of the orientation angle error and the distance to the line that
     * goes through the next waypoints.
     * The velocity is set to the maximum.
     */

    // Calculate target line from current to next waypoint (if there is any)
    double e_distance = calculateLineError();
    double e_angle = calculateAngleError();

    // TODO: summing entities with different units (metric and angular) is probably bad.
    float error = e_distance + e_angle;
    ROS_DEBUG_NAMED(MODULE, "OnLine: e_dist = %g, e_angle = %g  ==>  e_comb = %g",
                    e_distance, e_angle, error);

    // draw steer front
    if (visualizer_->hasSubscriber()) {
        visualizer_->drawSteeringArrow(path_driver_->getFixedFrameId(), 1, path_driver_->getRobotPoseMsg(), e_angle, 0.2, 1.0, 0.2);
        visualizer_->drawSteeringArrow(path_driver_->getFixedFrameId(), 2, path_driver_->getRobotPoseMsg(), e_distance, 0.2, 0.2, 1.0);
        visualizer_->drawSteeringArrow(path_driver_->getFixedFrameId(), 3, path_driver_->getRobotPoseMsg(), error, 1.0, 0.2, 0.2);
    }

    return error;
}

float RobotController_Ackermann_Pid::getErrorApproachSubpathEnd()
{
    /* The error is the sum of the orientation angle error and the sideways distance to the
     * waypoint (that is the distance on the y-axis in the robot frame)
     * The velocity is decreasing with the distance to the waypoint.
     */

    // Calculate target line from current to next waypoint (if there is any)
    double e_distance = calculateSidewaysDistanceError();
    double e_angle = calculateAngleError();

    // TODO: summing entities with different units (metric and angular) is probably bad.
    float error = e_distance + e_angle;
    ROS_DEBUG_NAMED(MODULE, "Approach: e_dist = %g, e_angle = %g  ==>  e_comb = %g",
                    e_distance, e_angle, error);

    if (visualizer_->hasSubscriber()) {
        visualizer_->drawCircle(2, ((geometry_msgs::Pose) path_->getCurrentWaypoint()).position,
                                0.5, getFixedFrame(), "turning point", 1, 1, 1);

        // draw steer front
        visualizer_->drawSteeringArrow(path_driver_->getFixedFrameId(), 1, path_driver_->getRobotPoseMsg(), e_angle, 0.2, 1.0, 0.2);
        visualizer_->drawSteeringArrow(path_driver_->getFixedFrameId(), 2, path_driver_->getRobotPoseMsg(), e_distance, 0.2, 0.2, 1.0);
        visualizer_->drawSteeringArrow(path_driver_->getFixedFrameId(), 3, path_driver_->getRobotPoseMsg(), error, 1.0, 0.2, 0.2);
    }

    return error;
}


void RobotController_Ackermann_Pid::updateCommand(float error)
{
    // call PID controller for steering.
    float u = 0;
    if (!steer_pid_.execute(error, &u)) {
        return; // Nothing to do
    }

    ROS_DEBUG_NAMED(MODULE, "PID: error = %g, u = %g", error, u);
    visualizer_->drawSteeringArrow(path_driver_->getFixedFrameId(), 14, path_driver_->getRobotPoseMsg(), u, 0.0, 1.0, 1.0);

    float steer = std::max(-opt_.max_steer(), std::min(u, opt_.max_steer()));
    ROS_DEBUG_STREAM_NAMED(MODULE, "direction = " << dir_sign_ << ", steer = " << steer);

    // Control velocity
    float velocity = controlVelocity(steer);

    cmd_.setDirection(dir_sign_ * steer);
    cmd_.setVelocity(dir_sign_ * velocity);
}

float RobotController_Ackermann_Pid::controlVelocity(float steer_angle) const
{
    PathFollowerParameters path_driver_opt = path_driver_->getOptions();
    float velocity = velocity_;

    if(abs(steer_angle) > path_driver_opt.steer_slow_threshold()) {
        ROS_INFO_STREAM_THROTTLE_NAMED(2, MODULE, "slowing down");
        velocity *= 0.75;
    }
//***todo rewrite

    // Reduce maximal velocity, when driving backwards.
    if(dir_sign_ < 0) {
        velocity = min(velocity, 0.4f * path_driver_opt.max_velocity());
    }

    // linearly reduce velocity, if the goal is within 2s*velocity (e.g. when driving with
    // 2 m/s, start to slow down 4m in front of the goal)
    // path_->getRemainingSubPathDistance() only returns the distance starting from the next
    // waypoint, so add the distance of the robot to this waypoint to get a more precise result.
    float distance_to_next_wp = std::sqrt(next_wp_local_.dot(next_wp_local_));
    float dist_to_path_end = path_->getRemainingSubPathDistance() + distance_to_next_wp;
    if (dist_to_path_end < 2*velocity) {
        velocity = std::max(0.1f + dist_to_path_end / 2.0f, path_driver_->getOptions().min_velocity());
    }
    //ROS_INFO("dist:      %f", dist_to_path_end);
    //ROS_INFO("v: %f", velocity);


    // make sure, the velocity is in the allowed range
    if (velocity < path_driver_opt.min_velocity()) {
        velocity = path_driver_opt.min_velocity();
        ROS_WARN_THROTTLE_NAMED(5, MODULE, "Velocity is below minimum. It is set to minimum velocity.");
    } else if (velocity > path_driver_opt.max_velocity()) {
        velocity = path_driver_opt.max_velocity();
        ROS_WARN_THROTTLE_NAMED(5, MODULE, "Velocity is above maximum. Reduce to maximum velocity.");
    }

    return velocity;
}

double RobotController_Ackermann_Pid::distanceToWaypoint(const Waypoint &wp) const
{
    Eigen::Vector3d pose = path_driver_->getRobotPose();
    return std::hypot(pose(0) - wp.x, pose(1) - wp.y);
}

void RobotController_Ackermann_Pid::predictPose(Vector2d &front_pred, Vector2d &rear_pred) const
{
    //NOTE: This is an ancient relict of the days of the `motion_control` package.
    // I am not absolutely sure, what it is doing. I think it has the purpose to predict the
    // positon of the robot (more exactly its front and rear axes) in the next time step.
    // This is how it is used at least, though I do not know if this is what it really does...
    // ~Felix

    double dt = opt_.dead_time(); //TODO: could opt_.pid_ta() be used instead?
    double deltaf = cmd_.getDirectionAngle();
    double deltar = 0.0; // currently not supported
    double v = 2 * last_velocity_; // why '2*'?

    double beta = std::atan(0.5 * (std::tan(deltaf) + std::tan(deltar)));
    double ds = v * dt;
    double dtheta = ds * std::cos(beta) * (std::tan(deltaf) - std::tan(deltar)) / opt_.l();
    double thetan = dtheta; // <- why this ???
    double yn = ds * std::sin(dtheta * 0.5 + beta * 0.5);
    double xn = ds * std::cos(dtheta * 0.5 + beta * 0.5);

    //ROS_DEBUG_NAMED(MODULE, "predict pose: dt = %g, deltaf = %g, deltar = %g, v = %g, "
    //                        "beta = %g, ds = %g, dtheta = %g, yn = %g, xn = %g",
    //                dt, deltaf, deltar, v, beta, ds, dtheta, yn, xn);

    front_pred[0] = xn + cos(thetan) * opt_.l()/2.0;
    front_pred[1] = yn + sin(thetan) * opt_.l()/2.0;
    rear_pred[0]  = xn - cos(thetan) * opt_.l()/2.0;
    rear_pred[1]  = yn - sin(thetan) * opt_.l()/2.0;

    ROS_DEBUG_STREAM_NAMED(MODULE, "predict pose. front: " << front_pred << ", rear: " << rear_pred);
}

double RobotController_Ackermann_Pid::calculateLineError() const
{
    geometry_msgs::PoseStamped followup_next_wp_map;
    followup_next_wp_map.header.stamp = ros::Time::now();

    if(path_->getWaypointIndex() + 1 == path_->getCurrentSubPath().size()) {
        followup_next_wp_map.pose = path_->getWaypoint(path_->getWaypointIndex() - 1);
    } else {
        followup_next_wp_map.pose = path_->getWaypoint(path_->getWaypointIndex() + 1);
    }

    Line2d target_line;
    Vector3d followup_next_wp_local;
    if (!path_driver_->transformToLocal( followup_next_wp_map, followup_next_wp_local)) {
        throw EmergencyBreakException("Cannot transform next waypoint",
                                      path_msgs::FollowPathResult::RESULT_STATUS_TF_FAIL);
    }
    target_line = Line2d( next_wp_local_.head<2>(), followup_next_wp_local.head<2>());
    visualizer_->visualizeLine(target_line);

    Vector2d main_carrot, alt_carrot, front_pred, rear_pred;
    predictPose(front_pred, rear_pred);
    if(dir_sign_ >= 0) {
        main_carrot = front_pred;
        alt_carrot = rear_pred;
    } else {
        main_carrot = rear_pred;
        alt_carrot = front_pred;
    }

    if (visualizer_->hasSubscriber()) {
        visualizeCarrot(main_carrot, 0, 1.0,0.0,0.0);
        visualizeCarrot(alt_carrot, 1, 0.0,0.0,0.0);
    }

    return -target_line.GetSignedDistance(main_carrot) - 0.25 * target_line.GetSignedDistance(alt_carrot);
}

double RobotController_Ackermann_Pid::calculateSidewaysDistanceError() const
{
    const double tolerance = 0.1;
    Vector2d main_carrot, alt_carrot, front_pred, rear_pred;

    predictPose(front_pred, rear_pred);
    if(dir_sign_ >= 0) {
        main_carrot = front_pred;
        alt_carrot = rear_pred;
    } else {
        main_carrot = rear_pred;
        alt_carrot = front_pred;
    }

    if (visualizer_->hasSubscriber()) {
        visualizeCarrot(main_carrot, 0, 1.0,0.0,0.0);
        visualizeCarrot(alt_carrot, 1, 0.0,0.0,0.0);
    }

    double dist_on_y_axis = next_wp_local_[1] - main_carrot[1];
    if(std::abs(dist_on_y_axis) < tolerance) {
        return 0;
    } else {
        return dist_on_y_axis;
    }
}

void RobotController_Ackermann_Pid::visualizeCarrot(const Vector2d &carrot,
                                                    int id, float r, float g, float b) const
{
    geometry_msgs::PoseStamped carrot_local;
    carrot_local.pose.position.x = carrot[0];
    carrot_local.pose.position.y = carrot[1];

    carrot_local.pose.orientation = tf::createQuaternionMsgFromYaw(0);
    geometry_msgs::PoseStamped carrot_map;
    if (path_driver_->transformToGlobal(carrot_local, carrot_map)) {
        visualizer_->drawMark(id, carrot_map.pose.position, "prediction", r,g,b);
    }
}
