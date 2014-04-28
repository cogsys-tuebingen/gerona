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

bool BehaviourDriveBase::isCollision(double course)
{
    // only check for collisions, while driving forward (there's no laser at the backside)
    return (controller_->getDirSign() > 0 && parent_.checkCollision(course));
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

PathWithPosition BehaviourDriveBase::getPathWithPosition()
{
    BehaviouralPathDriver::Options& opt = getOptions();
    Path& current_path = getSubPath(opt.path_idx);
    return PathWithPosition(&current_path, opt.wp_idx);
}




//##### BEGIN BehaviourOnLine

BehaviourOnLine::BehaviourOnLine(BehaviouralPathDriver& parent)
    : BehaviourDriveBase(parent)
{
}


void BehaviourOnLine::execute(int *status)
{
    status_ptr_ = status;

    getNextWaypoint();
    checkWaypointTimeout();

    controller_->behaveOnLine(getPathWithPosition());
}


void BehaviourOnLine::getNextWaypoint()
{
    BehaviouralPathDriver::Options& opt = getOptions();
    Path& current_path = getSubPath(opt.path_idx);

    assert(opt.wp_idx < (int) current_path.size());

    int last_wp_idx = current_path.size() - 1;

    double tolerance = opt.wp_tolerance_;

    if(controller_->getDirSign() < 0) {
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

    controller_->behaveAvoidObstacle(getPathWithPosition());
}



void BehaviourAvoidObstacle::getNextWaypoint()
{
    // TODO: improve!
    BehaviouralPathDriver::Options& opt = getOptions();
    Path& current_path = getSubPath(opt.path_idx);

    assert(opt.wp_idx < (int) current_path.size());

    int last_wp_idx = current_path.size() - 1;

    double tolerance = opt.wp_tolerance_;

    if(controller_->getDirSign() < 0) {
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

    //TODO: is this really model independent? I think with omnidrive, dir_sign_ has no meaning?!
    // check if the sign changes
    int dir_sign = sign(next_wp_local_.x());
    if(step_ > 0 && dir_sign != controller_->getDirSign()) {
        checkIfDone(true);
    }

    controller_->setDirSign(dir_sign);

    ++step_;

    // check if point is reached
    if(!checkIfDone()) {
        controller_->behaveApproachTurningPoint(getPathWithPosition());
    } else {
        // only set this in the else-case as in the if case, it is set by the controller
        *status_ptr_ = path_msgs::FollowPathResult::MOTION_STATUS_MOVING;
    }
}

bool BehaviourApproachTurningPoint::checkIfDone(bool done)
{
    BehaviouralPathDriver::Options& opt = getOptions();

    if(!waiting_) {
        //! Difference of current robot pose to the next waypoint.
        Vector2d delta;
        delta << next_wp_map_.pose.position.x - parent_.getSlamPoseMsg().position.x,
                next_wp_map_.pose.position.y - parent_.getSlamPoseMsg().position.y;

        if (controller_->getDirSign() < 0) {
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
        controller_->stopMotion();

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
