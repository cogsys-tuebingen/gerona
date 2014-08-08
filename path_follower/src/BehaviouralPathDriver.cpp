/*
 * BehaviouralPathDriver.cpp
 *
 *  Created on: Apr 15, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

/// HEADER
#include "BehaviouralPathDriver.h"

/// PROJECT
#include "pathfollower.h"
#include "behaviours.h"
#include <std_msgs/Int32MultiArray.h>

/// SYSTEM
#include <boost/foreach.hpp>
#include <Eigen/Core>
#include <utils_general/MathHelper.h>
#include <cmath>
#include <cxxabi.h>
#include <boost/assign.hpp>

using namespace Eigen;
using namespace path_msgs;


namespace beep {
static std::vector<int> OBSTACLE_IN_PATH = boost::assign::list_of(25)(25)(25);
}


/// BEHAVIOUR BASE
Path& BehaviouralPathDriver::Behaviour::getSubPath(unsigned index)
{
    return parent_.paths_[index];
}
int BehaviouralPathDriver::Behaviour::getSubPathCount() const
{
    return parent_.paths_.size();
}

PathFollower& BehaviouralPathDriver::Behaviour::getNode()
{
    return *parent_.node_;
}
double BehaviouralPathDriver::Behaviour::distanceTo(const Waypoint& wp)
{
    return hypot(parent_.slam_pose_(0) - wp.x, parent_.slam_pose_(1) - wp.y);
}
BehaviouralPathDriver::Command& BehaviouralPathDriver::Behaviour::getCommand()
{
    return parent_.current_command_;
}
VectorFieldHistogram& BehaviouralPathDriver::Behaviour::getVFH()
{
    return getNode().getVFH();
}
BehaviouralPathDriver::Options& BehaviouralPathDriver::Behaviour::getOptions()
{
    return parent_.options_;
}
//END Behaviour

/// STATES / BEHAVIOURS






namespace {
std::string name(BehaviouralPathDriver::Behaviour* b) {
    int status;
    return abi::__cxa_demangle(typeid(*b).name(),  0, 0, &status);
}
}


/// Controller class: BehaviouralPathDriver

BehaviouralPathDriver::BehaviouralPathDriver(PathFollower *node)
    : node_(node), private_nh_("~"), active_behaviour_(NULL), pending_error_(-1)
{
    last_beep_ = ros::Time::now();
    beep_pause_ = ros::Duration(2.0);

    visualizer_ = Visualizer::getInstance();

    beeper_ = private_nh_.advertise<std_msgs::Int32MultiArray>("/cmd_beep", 100);
    configure();
}

void BehaviouralPathDriver::start()
{
    options_.reset();

    clearActive();

    active_behaviour_ = new BehaviourOnLine(*this);
    ROS_INFO_STREAM("init with " << name(active_behaviour_));
}

void BehaviouralPathDriver::stop()
{
    clearActive();

    current_command_.velocity = 0;
}

int BehaviouralPathDriver::getType()
{
    //return FollowPathGoal::MOTION_FOLLOW_PATH;
    return 0; //TODO: change this, if different types are reimplemented.
}


int BehaviouralPathDriver::execute(FollowPathFeedback& feedback, FollowPathResult& result)
{
    /* TODO:
     * The global use of the result-constants as status codes is a bit problematic, as there are feedback
     * states, which do not imply that the path execution is finished (and thus there is no result to send).
     * This is currently the case for collisions.
     */

    // constants for return codes
    const int DONE = 0;
    const int MOVING = 1;

    // Pending error?
    if ( pending_error_ >= 0 ) {
        result.status = pending_error_;
        pending_error_ = -1;
        stop();

        return DONE;
    }

    if(paths_.empty()) {
        clearActive();
        result.status = FollowPathResult::MOTION_STATUS_SUCCESS;
        return DONE;
    }

    if(active_behaviour_ == NULL) {
        start();
    }

    if ( !node_->getWorldPose( &slam_pose_, &slam_pose_msg_ )) {
        stop(); // FIXME: stop() sets velocity to 0, but due to the return, this is never published.
        result.status = FollowPathResult::MOTION_STATUS_SLAM_FAIL;
        return DONE;
    }

    visualizer_->drawArrow(0, slam_pose_msg_, "slam pose", 2.0, 0.7, 1.0);


    int status = FollowPathResult::MOTION_STATUS_INTERNAL_ERROR;
    try {
        ROS_DEBUG_STREAM("executing " << name(active_behaviour_));
        active_behaviour_->execute(&status);

    } catch(NullBehaviour* null) {
        ROS_WARN_STREAM("stopping after " << name(active_behaviour_));
        clearActive();

        assert(status == FollowPathResult::MOTION_STATUS_SUCCESS);

        current_command_.velocity = 0;

    } catch(Behaviour* next_behaviour) {
        std::cout << "switching behaviour from " << name(active_behaviour_) << " to " << name(next_behaviour) << std::endl;
        clearActive();
        active_behaviour_ = next_behaviour;
    }

    getController()->publishCommand();

    if(status == FollowPathResult::MOTION_STATUS_COLLISION) {
        // collision is not aborting (the obstacle might be moving away)
        feedback.status = FollowPathFeedback::MOTION_STATUS_COLLISION;
        return MOVING;
    } else if (status == FollowPathResult::MOTION_STATUS_MOVING) {
        feedback.status = FollowPathFeedback::MOTION_STATUS_MOVING;
        return MOVING;
    } else if (active_behaviour_ != NULL) {
        ROS_INFO_STREAM("aborting, clearing active, status=" << status);
        clearActive();

        result.status = status;
        return DONE;
    }
    //else
    // I think status == FollowPathResult::MOTION_STATUS_SUCCESS should be the only possible case here.
    if (status != FollowPathResult::MOTION_STATUS_SUCCESS) ROS_ERROR("I thought wrong... File: %s, Line: %d", __FILE__, __LINE__);

    result.status = status;
    return DONE;
}

void BehaviouralPathDriver::configure()
{
    ros::NodeHandle nh("~");
    nh.param( "waypoint_tolerance", options_.wp_tolerance_, 0.20 );
    nh.param( "goal_tolerance", options_.goal_tolerance_, 0.15 );
    nh.param( "steer_slow_threshold", options_.steer_slow_threshold_, 0.25 );
    nh.param( "max_distance_to_path", options_.max_distance_to_path_, 0.3 ); //TODO: find reasonable default value.

    // use ros::param here, because nh.param can't handle floats...
    ros::param::param<float>( "~min_velocity", options_.min_velocity_, 0.4 );
    ros::param::param<float>( "~max_velocity", options_.max_velocity_, 2.0 );
    ros::param::param<float>( "~collision_box_width", options_.collision_box_width_, 0.5);
    ros::param::param<float>( "~collision_box_min_length", options_.collision_box_min_length_, 0.8);
    ros::param::param<float>( "~collision_box_crit_length", options_.collision_box_crit_length_, 0.3);
    ros::param::param<float>( "~collision_box_max_length", options_.collision_box_max_length_, 1.0);
    ros::param::param<float>( "~collision_box_velocity_factor", options_.collision_box_velocity_factor_, 1.0);
    ros::param::param<float>( "~collision_box_velocity_saturation", options_.collision_box_velocity_saturation_, options_.max_velocity_);

    if(options_.max_velocity_ < options_.min_velocity_) {
        ROS_ERROR("min velocity larger than max velocity!");
        options_.max_velocity_ = options_.min_velocity_;
    }
    if(options_.collision_box_max_length_ < options_.collision_box_min_length_) {
        ROS_ERROR("min length larger than max length!");
        options_.collision_box_min_length_ = options_.collision_box_max_length_;
    }
    if(options_.collision_box_min_length_ < options_.collision_box_crit_length_) {
        ROS_ERROR("min length smaller than crit length!");
        options_.collision_box_crit_length_ = options_.collision_box_min_length_;
    }
}

void BehaviouralPathDriver::setPath(const nav_msgs::Path& path)
{
    path_ = path;

    paths_.clear();

    findSegments(getController()->isOmnidirectional());

    getController()->reset();
}

void BehaviouralPathDriver::findSegments(bool only_one_segment)
{
    // find segments
    unsigned n = path_.poses.size();
    if(n < 2) {
        return;
    }

    Path current_segment;

    Waypoint last_point(path_.poses[0]);
    current_segment.push_back(last_point);

    int id = 0;

    for(unsigned i = 1; i < n; ++i){
        const Waypoint current_point(path_.poses[i]);

        // append to current segment
        current_segment.push_back(current_point);

        bool is_the_last_node = i == n-1;
        bool segment_ends_with_this_node = false;

        if(is_the_last_node) {
            // this is the last node
            segment_ends_with_this_node = true;

        } else {
            const Waypoint next_point(path_.poses[i+1]);

            // if angle between last direction and next direction to large -> segment ends
            double diff_last_x = current_point.x - last_point.x;
            double diff_last_y = current_point.y - last_point.y;
            double last_angle = std::atan2(diff_last_y, diff_last_x);

            double diff_next_x = next_point.x - current_point.x;
            double diff_next_y = next_point.y - current_point.y;
            double next_angle = std::atan2(diff_next_y, diff_next_x);

            double angle = MathHelper::AngleClamp(last_angle - next_angle);

            bool split_segment = std::abs(angle) > M_PI / 3.0;
            if(!only_one_segment && split_segment) {
                // new segment!
                // current node is the last one of the old segment
                segment_ends_with_this_node = true;
            }
        }

        visualizer_->drawArrow(id++, current_point, "paths", 0, 0, 0);
        if(segment_ends_with_this_node) {
            // Marker for subpaths
            visualizer_->drawMark(id++, ((geometry_msgs::Pose)current_point).position, "paths", 0.2,0.2,0.2);


            paths_.push_back(current_segment);
            current_segment.clear();

            if(!is_the_last_node) {
                // begin new segment
                // current node is also the first one of the new segment
                current_segment.push_back(current_point);
            }
        }

        last_point = current_point;
    }
}

void BehaviouralPathDriver::setGoal(const FollowPathGoal &goal)
{
    pending_error_ = -1;
    options_.velocity_ = goal.velocity;

    if ( goal.path.poses.size() < 2 ) {
        ROS_ERROR( "Got an invalid path with less than two poses." );
        stop();
        pending_error_ = FollowPathResult::MOTION_STATUS_INTERNAL_ERROR;
        return;
    }

    setPath(goal.path);

    ROS_INFO_STREAM("Following path with " << goal.path.poses.size() << " poses.");
}

void BehaviouralPathDriver::clearActive()
{
    if(active_behaviour_ != NULL) {
        delete active_behaviour_;
    }
    active_behaviour_ = NULL;
}


void BehaviouralPathDriver::beep(const std::vector<int> &beeps)
{
    ros::Time now = ros::Time::now();
    if(last_beep_ + beep_pause_ > now) {
        return;
    }

    last_beep_ = now;

    std_msgs::Int32MultiArray msg;

    msg.data.insert(msg.data.begin(), beeps.begin(), beeps.end());

    beeper_.publish(msg);
}

bool BehaviouralPathDriver::checkCollision(double course)
{
    //! Factor which defines, how much the box is enlarged in curves.
    const float enlarge_factor = 0.5; //TODO: should this be a parameter?

    /* Calculate length of the collision box, depending on current velocity.
     * v <= v_min:
     *   length = min_length
     * v > v_min && v < v_sat:
     *   length  interpolated between min_length and max_length:
     *   length = min_length + FACTOR * (max_length - min_length) * (v - v_min) / (v_sat - v_min)
     * v >= v_sat:
     *   length = max_length
     */
    float v = node_->getVelocity().linear.x;//current_command_.velocity;

    const float diff_to_min_velocity = v - options_.min_velocity_;

    const float norm = options_.collision_box_velocity_saturation_ - options_.min_velocity_;
    const float span = options_.collision_box_max_length_ - options_.collision_box_min_length_;
    const float interp = std::max(0.0f, diff_to_min_velocity) / std::max(norm, 0.001f);
    const float f = std::min(1.0f, options_.collision_box_velocity_factor_ * interp);

    float box_length = options_.collision_box_min_length_ + span * f;

    Path& current_path = paths_[options_.path_idx];
    double distance_to_goal = current_path.back().distanceTo(current_path[options_.wp_idx]);

    if(box_length > distance_to_goal) {
        box_length = distance_to_goal + 0.2;
    }

    if(box_length < options_.collision_box_crit_length_) {
        box_length = options_.collision_box_crit_length_;
    }


    bool collision = getNode()->checkCollision(course, box_length, options_.collision_box_width_, enlarge_factor);

    if(collision) {
        beep(beep::OBSTACLE_IN_PATH);
    }

    return collision;
}

RobotController *BehaviouralPathDriver::getController()
{
    return node_->getController();
}

PathFollower* BehaviouralPathDriver::getNode() const
{
    return node_;
}
