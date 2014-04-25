#include "behaviours.h"
#include "pathfollower.h"
#include <utils_general/MathHelper.h>

using namespace Eigen;

namespace {
double sign(double value) {
    if (value < 0) return -1;
    if (value > 0) return 1;
    return 0;
}
}

//##### BEGIN BehaviourDriveBase
BehaviourDriveBase::BehaviourDriveBase(BehaviouralPathDriver &parent)
    : Behaviour(parent)
{
    visualizer_ = Visualizer::getInstance();

    double wpto;
    ros::param::param<double>("~waypoint_timeout", wpto, 10.0);
    waypoint_timeout.duration = ros::Duration(wpto);
    waypoint_timeout.reset();
}

double BehaviourDriveBase::calculateAngleError()
{
    return MathHelper::AngleClamp(tf::getYaw(next_wp_map_.pose.orientation) - tf::getYaw(parent_.getSlamPoseMsg().orientation));
}

double BehaviourDriveBase::calculateLineError()
{
    BehaviouralPathDriver::Options& opt = getOptions();
    Path& current_path = getSubPath(opt.path_idx);

    geometry_msgs::PoseStamped followup_next_wp_map;
    followup_next_wp_map.header.stamp = ros::Time::now();

    if(opt.wp_idx + 1 == current_path.size()) {
        followup_next_wp_map.pose = current_path[opt.wp_idx - 1];
    } else {
        followup_next_wp_map.pose = current_path[opt.wp_idx + 1];
    }

    Line2d target_line;
    Vector3d followup_next_wp_local;
    if (!getNode().transformToLocal( followup_next_wp_map, followup_next_wp_local)) {
        *status_ptr_ = path_msgs::FollowPathResult::MOTION_STATUS_INTERNAL_ERROR;
        throw new BehaviourEmergencyBreak(parent_);
    }
    target_line = Line2d( next_wp_local_.head<2>(), followup_next_wp_local.head<2>());
    visualizeLine(target_line);

    Vector2d main_carrot, alt_carrot, front_pred, rear_pred;
    parent_.predictPose(front_pred, rear_pred);
    if(dir_sign_ >= 0) {
        main_carrot = front_pred;
        alt_carrot = rear_pred;
    } else {
        main_carrot = rear_pred;
        alt_carrot = front_pred;
    }

    visualizeCarrot(main_carrot, 0, 1.0,0.0,0.0);
    visualizeCarrot(alt_carrot, 1, 0.0,0.0,0.0);

    return -target_line.GetSignedDistance(main_carrot) - 0.25 * target_line.GetSignedDistance(alt_carrot);
}

double BehaviourDriveBase::calculateDistanceToCurrentPathSegment()
{
    /* Calculate line from last way point to current way point (which should be the line the robot is driving on)
     * and calculate the distance of the robot to this line.
     */

    BehaviouralPathDriver::Options& opt = getOptions();
    Path& current_path = getSubPath(opt.path_idx);

    assert(opt.wp_idx < (int) current_path.size());

    // opt.wp_idx should be the index of the next waypoint. The last waypoint ist then simply wp_idx - 1.
    // Usually wp_idx is greater than zero, so this is possible.
    // There are, however, situations where wp_idx = 0. In this case the segment starting in wp_idx is used rather
    // than the one ending there (I am not absolutly sure if this a good behaviour, so observe this via debug-output).
    int wp1_idx = 0;
    if (opt.wp_idx > 0) {
        wp1_idx = opt.wp_idx - 1;
    } else {
        // if wp_idx == 0, use the segment from wp_idx to the following waypoint.
        wp1_idx = opt.wp_idx + 1;

        ROS_DEBUG("Toggle waypoints as wp_idx == 0 in calculateDistanceToCurrentPathSegment() (%s, line %d)", __FILE__, __LINE__);
    }

    geometry_msgs::Pose wp1 = current_path[wp1_idx];
    geometry_msgs::Pose wp2 = current_path[opt.wp_idx];

    // line from last waypoint to current one.
    Line2d segment_line(Vector2d(wp1.position.x, wp1.position.y), Vector2d(wp2.position.x, wp2.position.y));

    ///// visualize start and end point of the current segment (for debugging)
    visualizer_->drawMark(24, wp1.position, "segment_marker", 0, 1, 1);
    visualizer_->drawMark(25, wp2.position, "segment_marker", 1, 0, 1);
    /////

    // get distance of robot (slam_pose_) to segment_line.
    return segment_line.GetDistance(parent_.getSlamPose().head<2>());
}

void BehaviourDriveBase::visualizeCarrot(const Vector2d &carrot, int id, float r, float g, float b)
{
    geometry_msgs::PoseStamped carrot_local;
    carrot_local.pose.position.x = carrot[0];
    carrot_local.pose.position.y = carrot[1];

    carrot_local.pose.orientation = tf::createQuaternionMsgFromYaw(0);
    geometry_msgs::PoseStamped carrot_map;
    if (getNode().transformToGlobal(carrot_local, carrot_map)) {
        visualizer_->drawMark(id, carrot_map.pose.position, "prediction", r,g,b);
    }
}

void BehaviourDriveBase::visualizeLine(const Line2d &line)
{
    Eigen::Vector2d from = line.GetOrigin() - 5 * line.GetDirection();
    Eigen::Vector2d to = line.GetOrigin() + 5 * line.GetDirection();

    geometry_msgs::Point f,t;
    f.x = from(0);
    f.y = from(1);
    t.x = to(0);
    t.y = to(1);

    visualizer_->drawLine(2, f, t, "/base_link", "line", 0.7, 0.2, 1.0);
}

double BehaviourDriveBase::calculateCourse()
{
    return getCommand().steer_front;
}

bool BehaviourDriveBase::isCollision(double course)
{
    // only check for collisions, while driving forward (there's no laser at the backside)
    return (dir_sign_ > 0 && parent_.checkCollision(course));
}

bool BehaviourDriveBase::setCommand(double error, double speed) //TODO: float would be sufficient for 'speed'
{
    BehaviouralPathDriver::Options& opt = getOptions();

    double delta_f_raw = 0;

    *status_ptr_ = path_msgs::FollowPathResult::MOTION_STATUS_MOVING;

    if ( !getPid().execute( error, delta_f_raw)) {
        // Nothing to do
        return false;
    }

    drawSteeringArrow(14, parent_.getSlamPoseMsg(), delta_f_raw, 0.0, 1.0, 1.0);

    double threshold = 5.0;
    double threshold_max_distance = 3.5 /*m*/;

    Path& current_path = getSubPath(opt.path_idx);
    double distance_to_goal = current_path.back().distanceTo(current_path[opt.wp_idx]);
    double threshold_distance = std::min(threshold_max_distance,
                                         std::max((double) opt.collision_box_min_length_, distance_to_goal));

    double delta_f;
    VectorFieldHistogram& vfh = getVFH();
    bool collision = false;

    // FIXME: check if ~use_vfh == true
    if(!vfh.isReady()) {
        ROS_WARN_THROTTLE(1, "Not using VFH, not ready yet! (Maybe obstacle map not published?)");
        delta_f = delta_f_raw;
    } else {
        vfh.create(threshold_distance, threshold);
        collision = !vfh.adjust(delta_f_raw, threshold, delta_f);
        vfh.visualize(delta_f_raw, threshold);
    }

    drawSteeringArrow(14, parent_.getSlamPoseMsg(), delta_f, 0.0, 1.0, 1.0);


    BehaviouralPathDriver::Command& cmd = getCommand();


    double steer = std::abs(delta_f);
    ROS_DEBUG_STREAM("dir=" << dir_sign_ << ", steer=" << steer);
    if(steer > getOptions().steer_slow_threshold_) {
        ROS_WARN_STREAM_THROTTLE(2, "slowing down");
        speed *= 0.5;
    }

    // make sure, the speed is in the allowed range
    if (speed < getOptions().min_velocity_) {
        speed = getOptions().min_velocity_;
        ROS_WARN_THROTTLE(5, "Velocity is below minimum. It is set to minimum velocity.");
    } else if (speed > getOptions().max_velocity_) {
        speed = getOptions().max_velocity_;
        ROS_WARN_THROTTLE(5, "Velocity is above maximum. Reduce to maximum velocity.");
    }

    cmd.steer_front = dir_sign_ * delta_f;
    cmd.steer_back = 0;

    collision |= isCollision(calculateCourse());

    if(collision) {
        ROS_WARN_THROTTLE(1, "Collision!");
        *status_ptr_ = path_msgs::FollowPathResult::MOTION_STATUS_COLLISION; //FIXME: not so good to use result-constant if it is not finishing the action...

        getCommand().velocity = 0;
        parent_.publishCommand();

    } else {
        cmd.velocity = dir_sign_ * speed;
    }

    ROS_DEBUG("Set velocity to %g", speed);
    return (std::abs(delta_f - delta_f_raw) > 0.05);
}

void BehaviourDriveBase::drawSteeringArrow(int id, geometry_msgs::Pose steer_arrow, double angle, double r, double g, double b)
{
    steer_arrow.orientation = tf::createQuaternionMsgFromYaw(tf::getYaw(steer_arrow.orientation) + angle);
    visualizer_->drawArrow(id, steer_arrow, "steer", r, g, b);
}

void BehaviourDriveBase::setStatus(int status)
{
    *status_ptr_ = status;
}

void BehaviourDriveBase::checkWaypointTimeout()
{
    if (waypoint_timeout.isExpired()) {
        ROS_WARN("Waypoint Timeout! The robot did not reach the next waypoint within %g sec. Abort path execution.",
                 waypoint_timeout.duration.toSec());
        *status_ptr_ = path_msgs::FollowPathResult::MOTION_STATUS_TIMEOUT;
        throw new BehaviourEmergencyBreak(parent_);
    }
}




//##### BEGIN BehaviourOnLine

BehaviourOnLine::BehaviourOnLine(BehaviouralPathDriver& parent)
    : BehaviourDriveBase(parent)
{
    getPid().reset();
}


void BehaviourOnLine::execute(int *status)
{
    status_ptr_ = status;

    getNextWaypoint();
    checkWaypointTimeout();

    dir_sign_ = sign(next_wp_local_.x());

    // Calculate target line from current to next waypoint (if there is any)
    double e_distance = calculateLineError();
    double e_angle = calculateAngleError();

    double e_combined = e_distance + e_angle;

    // draw steer front
    drawSteeringArrow(1, parent_.getSlamPoseMsg(), e_angle, 0.2, 1.0, 0.2);
    drawSteeringArrow(2, parent_.getSlamPoseMsg(), e_distance, 0.2, 0.2, 1.0);
    drawSteeringArrow(3, parent_.getSlamPoseMsg(), e_combined, 1.0, 0.2, 0.2);

    float speed = getOptions().velocity_;

    // TODO: better speed control
    //       - backwards not slower, but lower max speed
    //       - slower when close to goal, similar to ApproachGoal
    if(dir_sign_ < 0) {
        speed *= 0.5;
    }

    // if the robot is in this state, we assume that it is not avoiding any obstacles
    // so we abort, if robot moves too far from the path
    if (calculateDistanceToCurrentPathSegment() > getOptions().max_distance_to_path_) {
        std::stringstream cmd;
        cmd << "espeak \"" << "abort: too far away!" << "\" 2> /dev/null 1> /dev/null &";
        system(cmd.str().c_str());

        ROS_WARN("Moved too far away from the path. Abort.");
        *status_ptr_ = path_msgs::FollowPathResult::MOTION_STATUS_PATH_LOST;
        throw new BehaviourEmergencyBreak(parent_);
    }

    if(setCommand(e_combined, speed)) {
        *status_ptr_ = path_msgs::FollowPathResult::MOTION_STATUS_MOVING;
        throw new BehaviourAvoidObstacle(parent_);
    }
}


void BehaviourOnLine::getNextWaypoint()
{
    BehaviouralPathDriver::Options& opt = getOptions();
    Path& current_path = getSubPath(opt.path_idx);

    assert(opt.wp_idx < (int) current_path.size());

    int last_wp_idx = current_path.size() - 1;

    double tolerance = opt.wp_tolerance_;

    if(dir_sign_ < 0) {
        tolerance *= 2;
    }

    // if distance to wp < threshold
    while(distanceTo(current_path[opt.wp_idx]) < tolerance) {
        if(opt.wp_idx >= last_wp_idx) {
            // if distance to wp == last_wp -> state = APPROACH_TURNING_POINT
            *status_ptr_ = path_msgs::FollowPathResult::MOTION_STATUS_MOVING;
            throw new BehaviourApproachTurningPoint(parent_);
        }
        else {
            // else choose next wp
            opt.wp_idx++;

            waypoint_timeout.reset();
        }
    }

    visualizer_->drawArrow(0, current_path[opt.wp_idx], "current waypoint", 1, 1, 0);
    visualizer_->drawArrow(1, current_path[last_wp_idx], "current waypoint", 1, 0, 0);

    next_wp_map_.pose = current_path[opt.wp_idx];
    next_wp_map_.header.stamp = ros::Time::now();

    if ( !getNode().transformToLocal( next_wp_map_, next_wp_local_ )) {
        *status_ptr_ = path_msgs::FollowPathResult::MOTION_STATUS_SLAM_FAIL;
        throw new BehaviourEmergencyBreak(parent_);
    }
}



//##### BEGIN BehaviourAvoidObstacle
void BehaviourAvoidObstacle::execute(int *status)
{
    status_ptr_ = status;

    getNextWaypoint();
    checkWaypointTimeout();

    dir_sign_ = sign(next_wp_local_.x());

    // Calculate target line from current to next waypoint (if there is any)
    double e_distance = calculateLineError();
    double e_angle = calculateAngleError();

    double e_combined = e_distance + e_angle;

    // draw steer front
    drawSteeringArrow(1, parent_.getSlamPoseMsg(), e_angle, 0.2, 1.0, 0.2);
    drawSteeringArrow(2, parent_.getSlamPoseMsg(), e_distance, 0.2, 0.2, 1.0);
    drawSteeringArrow(3, parent_.getSlamPoseMsg(), e_combined, 1.0, 0.2, 0.2);

    float speed = getOptions().velocity_;

    if(dir_sign_ < 0) {
        speed *= 0.5;
    }

    if(!setCommand(e_combined, speed)) {
        *status_ptr_ = path_msgs::FollowPathResult::MOTION_STATUS_MOVING;
        throw new BehaviourOnLine(parent_);
    }
}



void BehaviourAvoidObstacle::getNextWaypoint()
{
    // TODO: improve!
    BehaviouralPathDriver::Options& opt = getOptions();
    Path& current_path = getSubPath(opt.path_idx);

    assert(opt.wp_idx < (int) current_path.size());

    int last_wp_idx = current_path.size() - 1;

    double tolerance = opt.wp_tolerance_;

    if(dir_sign_ < 0) {
        tolerance *= 2;
    }

    // if distance to wp < threshold
    while(distanceTo(current_path[opt.wp_idx]) < tolerance) {
        if(opt.wp_idx >= last_wp_idx) {
            // if distance to wp == last_wp -> state = APPROACH_TURNING_POINT
            *status_ptr_ = path_msgs::FollowPathResult::MOTION_STATUS_MOVING;
            throw new BehaviourApproachTurningPoint(parent_);
        }
        else {
            // else choose next wp
            opt.wp_idx++;

            waypoint_timeout.reset();
        }
    }

    visualizer_->drawArrow(0, current_path[opt.wp_idx], "current waypoint", 1, 1, 0);
    visualizer_->drawArrow(1, current_path[last_wp_idx], "current waypoint", 1, 0, 0);

    next_wp_map_.pose = current_path[opt.wp_idx];
    next_wp_map_.header.stamp = ros::Time::now();

    if ( !getNode().transformToLocal( next_wp_map_, next_wp_local_ )) {
        *status_ptr_ = path_msgs::FollowPathResult::MOTION_STATUS_SLAM_FAIL;
        throw new BehaviourEmergencyBreak(parent_);
    }
}


//##### BEGIN BehaviourApproachTurningPoint

BehaviourApproachTurningPoint::BehaviourApproachTurningPoint(BehaviouralPathDriver &parent)
    : BehaviourDriveBase(parent), step_(0), waiting_(false)
{
}

void BehaviourApproachTurningPoint::execute(int *status)
{
    status_ptr_ = status;

    getNextWaypoint();
    checkWaypointTimeout();

    // check if the sign changes
    int dir_sign = sign(next_wp_local_.x());
    if(step_ > 0 && dir_sign != dir_sign_) {
        checkIfDone(true);
    }

    dir_sign_ = dir_sign;

    ++step_;

    // check if point is reached
    if(!checkIfDone()) {
        // Calculate target line from current to next waypoint (if there is any)
        double e_distance = calculateDistanceError();
        double e_angle = calculateAngleError();

        double e_combined = e_distance + e_angle;

        visualizer_->drawCircle(2, next_wp_map_.pose.position, 0.5, "/map", "turning point", 1, 1, 1);

        // draw steer front
        drawSteeringArrow(1, parent_.getSlamPoseMsg(), e_angle, 0.2, 1.0, 0.2);
        drawSteeringArrow(2, parent_.getSlamPoseMsg(), e_distance, 0.2, 0.2, 1.0);
        drawSteeringArrow(3, parent_.getSlamPoseMsg(), e_combined, 1.0, 0.2, 0.2);

        double distance = std::sqrt(next_wp_local_.dot(next_wp_local_));
        double velocity = std::min(0.1 + distance / 2.0, (double) getOptions().min_velocity_);

        setCommand(e_combined, velocity);
    } else {
        // only set this in the else-case as setCommand() in the if case sets the status
        *status_ptr_ = path_msgs::FollowPathResult::MOTION_STATUS_MOVING;
    }
}

double BehaviourApproachTurningPoint::calculateDistanceError()
{
    Vector2d main_carrot, alt_carrot, front_pred, rear_pred;
    parent_.predictPose(front_pred, rear_pred);
    if(dir_sign_ >= 0) {
        main_carrot = front_pred;
        alt_carrot = rear_pred;
    } else {
        main_carrot = rear_pred;
        alt_carrot = front_pred;
    }

    visualizeCarrot(main_carrot, 0, 1.0,0.0,0.0);
    visualizeCarrot(alt_carrot, 1, 0.0,0.0,0.0);

    Vector2d delta = next_wp_local_.head<2>() - main_carrot;

    if(std::abs(delta(1)) < 0.1) {
        return 0;
    }

    return delta(1);
}

bool BehaviourApproachTurningPoint::checkIfDone(bool done)
{
    BehaviouralPathDriver::Options& opt = getOptions();

    if(!waiting_) {
        //! Difference of current robot pose to the next waypoint.
        Vector2d delta;
        delta << next_wp_map_.pose.position.x - parent_.getSlamPoseMsg().position.x,
                next_wp_map_.pose.position.y - parent_.getSlamPoseMsg().position.y;

        if (dir_sign_ < 0) {
            delta *= -1;
        }

        Path& current_path = getSubPath(opt.path_idx);

        //! Unit vector pointing in the direction of the next waypoints orientation.
        Vector2d target_dir;
        //NOTE: current_path[opt.wp_idx] == next_wp_map_ ??
        target_dir << std::cos(current_path[opt.wp_idx].theta), std::sin(current_path[opt.wp_idx].theta);

        // atan2(y,x) = angle of the vector.
        //! Angle between the line from robot to waypoint and the waypoints orientation (only used for output?)
        double angle = MathHelper::AngleClamp(std::atan2(delta(1), delta(0)) - std::atan2(target_dir(1), target_dir(0)));

        ROS_WARN_STREAM_THROTTLE(1, "angle = " << angle);

        //        bool done = std::abs(angle) >= M_PI / 2;
        done |= delta.dot(target_dir) < 0;  // done, if angle is greater than 90°?!
    }

    if(done || waiting_) {
        getCommand().velocity = 0;
        getCommand().steer_front = 0;
        getCommand().steer_back= 0;

        parent_.publishCommand();

        if(std::abs(parent_.getNode()->getVelocity().linear.x) > 0.01) {
            ROS_WARN_THROTTLE(1, "WAITING until no more motion");
            waiting_ = true;
            return true;
        } else {
            done = true;
        }

        opt.path_idx++;
        opt.wp_idx = 0;

        if(opt.path_idx < getSubPathCount()) {
            *status_ptr_ = path_msgs::FollowPathResult::MOTION_STATUS_MOVING;
            throw new BehaviourOnLine(parent_);

        } else {
            *status_ptr_ = path_msgs::FollowPathResult::MOTION_STATUS_SUCCESS;
            throw new BehaviouralPathDriver::NullBehaviour;
        }
    }

    return done;
}

void BehaviourApproachTurningPoint::getNextWaypoint()
{
    BehaviouralPathDriver::Options& opt = getOptions();
    Path& current_path = getSubPath(opt.path_idx);

    assert(opt.wp_idx < (int) current_path.size());

    int last_wp_idx = current_path.size() - 1;
    opt.wp_idx = last_wp_idx;

    visualizer_->drawArrow(0, current_path[opt.wp_idx], "current waypoint", 1, 1, 0);
    visualizer_->drawArrow(1, current_path[last_wp_idx], "current waypoint", 1, 0, 0);

    next_wp_map_.pose = current_path[opt.wp_idx];
    next_wp_map_.header.stamp = ros::Time::now();

    if ( !getNode().transformToLocal( next_wp_map_, next_wp_local_ )) {
        *status_ptr_ = path_msgs::FollowPathResult::MOTION_STATUS_SLAM_FAIL;
        throw new BehaviourEmergencyBreak(parent_);
    }
}
