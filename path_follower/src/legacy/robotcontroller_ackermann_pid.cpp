#include <path_follower/legacy/robotcontroller_ackermann_pid.h>

/// STL
#include <sstream>
#include <cxxabi.h>

/// ROS
#include <ros/ros.h>

/// PROJECT
#include <path_follower/pathfollower.h>
#include <path_follower/legacy/behaviours.h>
#include <utils_general/MathHelper.h>
#include <path_msgs/FollowPathAction.h>
#include <path_follower/utils/path_exceptions.h>

using namespace Eigen;
using namespace std;
using namespace path_msgs;

namespace {
double sign(double value) {
    if (value < 0) return -1.0;
    if (value > 0) return 1.0;
    return 0.0;
}
}

namespace {
std::string name(Behaviour* b) {
    int status;
    return abi::__cxa_demangle(typeid(*b).name(),  0, 0, &status);
}
}



RobotController_Ackermann_Pid::RobotController_Ackermann_Pid(PathFollower *path_driver):
    RobotController(path_driver),
    active_behaviour_(NULL),
    filtered_speed_(0.0f)
{
    configure();

    visualizer_ = Visualizer::getInstance();
}

void RobotController_Ackermann_Pid::configure()
{
    opt_.printAllInstances();
    pid_.configure(opt_.pid_kp(),
                   opt_.pid_ki(),
                   opt_.pid_i_max(),
                   opt_.pid_delta_max() * M_PI/180.0,
                   opt_.pid_e_max(),
                   0.5,
                   opt_.pid_ta());
}

bool RobotController_Ackermann_Pid::setCommand(double error, float speed)
{
    PathFollowerParameters path_driver_opt = path_driver_->getOptions();

    double delta_f_raw = 0;

    setStatus(path_msgs::FollowPathResult::MOTION_STATUS_MOVING);

    if (!pid_.execute( error, delta_f_raw)) {
        // Nothing to do
        return false;
    }
    ROS_DEBUG("PID: error = %g, df = %g", error, delta_f_raw);

    visualizer_->drawSteeringArrow(14, path_driver_->getRobotPoseMsg(), delta_f_raw, 0.0, 1.0, 1.0);

    double delta_f = delta_f_raw;
//    visualizer_->drawSteeringArrow(14, path_driver_->getRobotPoseMsg(), delta_f, 0.0, 1.0, 1.0);


    double steer = std::abs(delta_f);
    ROS_DEBUG_STREAM("dir=" << dir_sign_ << ", steer=" << steer);
    if(steer > path_driver_opt.steer_slow_threshold()) {
        ROS_WARN_STREAM_THROTTLE(2, "slowing down");
        speed *= 0.5;
    }

    // make sure, the speed is in the allowed range
    if (speed < path_driver_opt.min_velocity()) {
        speed = path_driver_opt.min_velocity();
        ROS_WARN_THROTTLE(5, "Velocity is below minimum. It is set to minimum velocity.");
    } else if (speed > path_driver_opt.max_velocity()) {
        speed = path_driver_opt.max_velocity();
        ROS_WARN_THROTTLE(5, "Velocity is above maximum. Reduce to maximum velocity.");
    }

    cmd_.steer_front = dir_sign_ * delta_f;
    cmd_.steer_back = 0;
    cmd_.velocity = dir_sign_ * speed;

    ROS_DEBUG("Set velocity to %g", cmd_.velocity);
    return (std::abs(delta_f - delta_f_raw) > 0.05);
}

void RobotController_Ackermann_Pid::stopMotion()
{
    //FIXME: this method should be improved (Command could implement cast to MoveCommand)

    cmd_.velocity    = 0.0;
    cmd_.steer_front = 0.0;
    cmd_.steer_back  = 0.0;

    MoveCommand mcmd;
    mcmd.setVelocity(0);
    publishMoveCommand(mcmd);
}

void RobotController_Ackermann_Pid::reset()
{
    if(active_behaviour_ != NULL) {
        delete active_behaviour_;
    }
    active_behaviour_ = NULL;
}

void RobotController_Ackermann_Pid::start()
{
    active_behaviour_ = new BehaviourOnLine(*path_driver_);
    ROS_INFO_STREAM("init with " << name(active_behaviour_));
}

void RobotController_Ackermann_Pid::switchBehaviour(Behaviour* next_behaviour)
{
    reset();
    active_behaviour_ = next_behaviour;
}

void RobotController_Ackermann_Pid::initOnLine()
{
    pid_.reset();
}

void RobotController_Ackermann_Pid::behaveOnLine()
{
    dir_sign_ = sign(next_wp_local_.x());

    // Calculate target line from current to next waypoint (if there is any)
    double e_distance = calculateLineError();
    double e_angle = calculateAngleError();

    double e_combined = e_distance + e_angle;
    ROS_DEBUG("OnLine: e_dist = %g, e_angle = %g  ==>  e_comb = %g", e_distance, e_angle, e_combined);

    // draw steer front
    if (visualizer_->hasSubscriber()) {
        visualizer_->drawSteeringArrow(1, path_driver_->getRobotPoseMsg(), e_angle, 0.2, 1.0, 0.2);
        visualizer_->drawSteeringArrow(2, path_driver_->getRobotPoseMsg(), e_distance, 0.2, 0.2, 1.0);
        visualizer_->drawSteeringArrow(3, path_driver_->getRobotPoseMsg(), e_combined, 1.0, 0.2, 0.2);
    }

    float speed = velocity_;

    // TODO: better speed control
    //       - backwards not slower, but lower max speed
    //       - slower when close to goal, similar to ApproachGoal
    if(dir_sign_ < 0) {
        //speed *= 0.5;
    }

    if(setCommand(e_combined, speed)) {
        setStatus(path_msgs::FollowPathResult::MOTION_STATUS_MOVING);
    }
}

void RobotController_Ackermann_Pid::initApproachTurningPoint()
{
    atp_step_ = 0;
}

bool RobotController_Ackermann_Pid::behaveApproachTurningPoint()
{
    // CHECK IF DONE
    float dir_sign = sign(next_wp_local_.x());
    if(atp_step_++ > 0 && dir_sign != getDirSign()) {
        return true;
    }
    setDirSign(dir_sign);


    // Calculate target line from current to next waypoint (if there is any)
    double e_distance = calculateDistanceError();
    double e_angle = calculateAngleError();

    double e_combined = e_distance + e_angle;
    ROS_DEBUG("Approach: e_dist = %g, e_angle = %g  ==>  e_comb = %g", e_distance, e_angle, e_combined);

    if (visualizer_->hasSubscriber()) {
        visualizer_->drawCircle(2, ((geometry_msgs::Pose) path_->getCurrentWaypoint()).position, 0.5, "/map", "turning point", 1, 1, 1);

        // draw steer front
        visualizer_->drawSteeringArrow(1, path_driver_->getRobotPoseMsg(), e_angle, 0.2, 1.0, 0.2);
        visualizer_->drawSteeringArrow(2, path_driver_->getRobotPoseMsg(), e_distance, 0.2, 0.2, 1.0);
        visualizer_->drawSteeringArrow(3, path_driver_->getRobotPoseMsg(), e_combined, 1.0, 0.2, 0.2);
    }

    float distance = std::sqrt(next_wp_local_.dot(next_wp_local_));
    float velocity = std::max(0.1f + distance / 2.0f, path_driver_->getOptions().min_velocity());

    setCommand(e_combined, velocity);

    return false;
}

RobotController::MoveCommandStatus RobotController_Ackermann_Pid::computeMoveCommand(MoveCommand *cmd)
{
    try {
//        ROS_DEBUG_STREAM("executing " << name(active_behaviour_));
        int status = FollowPathFeedback::MOTION_STATUS_MOVING;
        Behaviour* next_behaviour = active_behaviour_->execute(&status);

        if(next_behaviour == NULL) {
            switchBehaviour(NULL);
            return MC_REACHED_GOAL;
        }

        if(active_behaviour_ != next_behaviour) {
            ROS_INFO_STREAM("switching behaviour from " << name(active_behaviour_)
                            << " to " << name(next_behaviour));
            switchBehaviour(next_behaviour);
        }

        // Quickfix: simply convert ackermann command to move command
        cmd->setDirection(cmd_.steer_front);
        cmd->setVelocity(cmd_.velocity);

        filtered_speed_ = cmd_.velocity;

        switch(status) {
        case FollowPathFeedback::MOTION_STATUS_MOVING:
            return MC_OKAY;
        default:
            ROS_WARN_STREAM("unknown status: " << status);
            return MC_ERROR;
        }
    } catch(const std::exception& e) {
        ROS_ERROR_STREAM("uncaught exception: " << e.what() << " => abort");
        reset();
        return MC_ERROR;
    }
}

void RobotController_Ackermann_Pid::publishMoveCommand(const MoveCommand &cmd) const
{
    geometry_msgs::Twist msg;
    //msg.linear.x  = velocity;
    //msg.angular.z = steer_front;
    msg.linear.x  = cmd.getVelocity();
    msg.angular.z = cmd.getDirectionAngle();

    cmd_pub_.publish(msg);
}

void RobotController_Ackermann_Pid::predictPose(Vector2d &front_pred, Vector2d &rear_pred)
{
    //TODO: revise this code

    double dt = opt_.dead_time();
    double deltaf = cmd_.steer_front;
    double deltar = cmd_.steer_back;
    double v = 2*filtered_speed_;

    double beta = std::atan(0.5*(std::tan(deltaf)+std::tan(deltar)));
    double ds = v*dt;
    double dtheta = ds*std::cos(beta)*(std::tan(deltaf)-std::tan(deltar))/opt_.l();
    double thetan = dtheta; //TODO <- why this ???
    double yn = ds*std::sin(dtheta*0.5+beta*0.5);
    double xn = ds*std::cos(dtheta*0.5+beta*0.5);

    ROS_DEBUG("predict pose: dt = %g, deltaf = %g, deltar = %g, v = %g, beta = %g, ds = %g, dtheta = %g, yn = %g, xn = %g",
              dt, deltaf, deltar, v, beta, ds, dtheta, yn, xn);
    // output when nan-values occured (hopefully fixed with commit 8bad8fd)
    // step 1: dt = 0.1, deltaf = 9.75024e+199, deltar = 4.26137e+257, v = 0,   beta = 0.308293, ds = 0,   dtheta = 0,   yn = 0,    xn = 0
    // step 2: dt = 0.1, deltaf = 9.75024e+199, deltar = 4.26137e+257, v = inf, beta = 0.308293, ds = inf, dtheta = inf, yn = -nan, xn = -nan

    front_pred[0] = xn+cos(thetan)*opt_.l()/2.0;
    front_pred[1] = yn+sin(thetan)*opt_.l()/2.0;
    rear_pred[0] = xn-cos(thetan)*opt_.l()/2.0;
    rear_pred[1] = yn-sin(thetan)*opt_.l()/2.0;

    ROS_DEBUG_STREAM("predict pose. front: " << front_pred << ", rear: " << rear_pred);
}

double RobotController_Ackermann_Pid::calculateCourse()
{
    return cmd_.steer_front;
}

double RobotController_Ackermann_Pid::calculateLineError()
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
        setStatus(path_msgs::FollowPathResult::MOTION_STATUS_INTERNAL_ERROR);
        throw new EmergencyBreakException("Cannot transform next waypoint");
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

double RobotController_Ackermann_Pid::calculateDistanceError()
{
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

    Vector2d delta = next_wp_local_.head<2>() - main_carrot;

    if(std::abs(delta(1)) < 0.1) {
        return 0;
    }

    return delta(1);
}

void RobotController_Ackermann_Pid::visualizeCarrot(const Vector2d &carrot, int id, float r, float g, float b)
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
