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

/// SYSTEM
#include <boost/foreach.hpp>
#include <Eigen/Core>
#include <visualization_msgs/Marker.h>
#include <utils_general/MathHelper.h>
#include <cmath>
#include <cxxabi.h>

using namespace motion_control;
using namespace Eigen;
using namespace path_msgs;


/// BEHAVIOUR BASE
BehaviouralPathDriver::Path& BehaviouralPathDriver::Behaviour::getSubPath(unsigned index)
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
PidCtrl& BehaviouralPathDriver::Behaviour::getPid()
{
    return parent_.pid_;
}
BehaviouralPathDriver::Command& BehaviouralPathDriver::Behaviour::getCommand()
{
    return parent_.current_command_;
}
VectorFieldHistogram& BehaviouralPathDriver::Behaviour::getVFH()
{
    return parent_.vfh_;
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

BehaviouralPathDriver::BehaviouralPathDriver(ros::Publisher &cmd_pub, PathFollower *node)
    : node_(node), private_nh_("~"), cmd_pub_(cmd_pub), active_behaviour_(NULL), pending_error_(-1)
{
    private_nh_.param("use_obstacle_map", use_obstacle_map_, false);

    if(use_obstacle_map_) {
        obstacle_map_sub_ = private_nh_.subscribe<nav_msgs::OccupancyGrid>("/obstacle_map", 0, boost::bind(&BehaviouralPathDriver::obstacleMapCallback, this, _1));
        //obstacle_map_sub_ = private_nh_.subscribe<nav_msgs::OccupancyGrid>("/obstacle_map", 0, boost::bind(&ObstacleDetector::gridMapCallback, &obstacle_detector_, _1));

    } else {
        laser_sub_ = private_nh_.subscribe<sensor_msgs::LaserScan>("/scan", 10, boost::bind(&BehaviouralPathDriver::laserCallback, this, _1));
    }


    vis_pub_ = private_nh_.advertise<visualization_msgs::Marker>("/marker", 100);
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

    geometry_msgs::Pose slampose_p;
    if ( !node_->getWorldPose( slam_pose_, &slampose_p )) {
        stop();
        result.status = FollowPathResult::MOTION_STATUS_SLAM_FAIL;
        return DONE;
    }

    drawArrow(0, slampose_p, "slam pose", 2.0, 0.7, 1.0);


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

    publishCommand();

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
    nh.param( "dead_time", options_.dead_time_, 0.10 );
    nh.param( "waypoint_tolerance", options_.wp_tolerance_, 0.20 );
    nh.param( "goal_tolerance", options_.goal_tolerance_, 0.15 );
    nh.param( "l", options_.l_, 0.38 );
    nh.param( "steer_slow_threshold", options_.steer_slow_threshold_, 0.25 );
    nh.param( "max_distance_to_path", options_.max_distance_to_path_, 0.3 ); //TODO: find reasonable default value.

    // use ros::param here, because nh.param can't handle floats...
    ros::param::param<float>( "~min_velocity", options_.min_velocity_, 0.4 );
    ros::param::param<float>( "~max_velocity", options_.max_velocity_, 2.0 );
    ros::param::param<float>( "~collision_box_width", options_.collision_box_width_, 0.5);
    ros::param::param<float>( "~collision_box_min_length", options_.collision_box_min_length_, 0.3);
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

    double ta, kp, ki, i_max, delta_max, e_max;
    nh.param( "pid/ta", ta, 0.03 );
    nh.param( "pid/kp", kp, 1.5 );
    nh.param( "pid/ki", ki, 0.001 );
    nh.param( "pid/i_max", i_max, 0.0 );
    nh.param( "pid/delta_max", delta_max, 30.0 );
    nh.param( "pid/e_max", e_max, 0.10 );

    pid_.configure( kp, ki, i_max, M_PI*delta_max/180.0, e_max, 0.5, ta );
}

void BehaviouralPathDriver::setPath(const nav_msgs::Path& path)
{
    path_ = path;

    paths_.clear();

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

            if(std::abs(angle) > M_PI / 3.0) {
                // new segment!
                // current node is the last one of the old segment
                segment_ends_with_this_node = true;
            }
        }

        drawArrow(id++, current_point, "paths", 0, 0, 0);
        if(segment_ends_with_this_node) {
            // Marker for subpaths
            drawMark(id++, ((geometry_msgs::Pose)current_point).position, "paths", 0.2,0.2,0.2);


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

void BehaviouralPathDriver::predictPose(Vector2d &front_pred, Vector2d &rear_pred)
{
    double dt = options_.dead_time_;
    double deltaf = current_command_.steer_front;
    double deltar = current_command_.steer_back;
    double v = 2*getFilteredSpeed();

    double beta = std::atan(0.5*(std::tan(deltaf)+std::tan(deltar)));
    double ds = v*dt;
    double dtheta = ds*std::cos(beta)*(std::tan(deltaf)-std::tan(deltar))/options_.l_;
    double thetan = dtheta; //TODO <- why this ???
    double yn = ds*std::sin(dtheta*0.5+beta*0.5);
    double xn = ds*std::cos(dtheta*0.5+beta*0.5);

    front_pred[0] = xn+cos(thetan)*options_.l_/2.0;
    front_pred[1] = yn+sin(thetan)*options_.l_/2.0;
    rear_pred[0] = xn-cos(thetan)*options_.l_/2.0;
    rear_pred[1] = yn-sin(thetan)*options_.l_/2.0;
}

void BehaviouralPathDriver::drawLine(int id, const geometry_msgs::Point &from, const geometry_msgs::Point &to, const std::string& frame,
                                     const std::string& ns, float r, float g, float b, double live, float scale)
{
    visualization_msgs::Marker marker;
    marker.ns = ns;
    marker.header.frame_id = frame;
    marker.header.stamp = ros::Time();
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = id;
    marker.lifetime = ros::Duration(live);
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;
    marker.pose.orientation.w = 1.0;
    marker.points.push_back(from);
    marker.points.push_back(to);
    marker.scale.x = scale;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    vis_pub_.publish(marker);
}

void BehaviouralPathDriver::drawCircle(int id, const geometry_msgs::Point &center, double radius, const string &frame, const std::string& ns, float r, float g, float b, double live)
{
    visualization_msgs::Marker marker;
    marker.ns = ns;
    marker.header.frame_id = frame;
    marker.header.stamp = ros::Time();
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = id;
    marker.lifetime = ros::Duration(live);
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;
    marker.pose.position = center;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = radius;
    marker.scale.y = radius;
    marker.scale.z = 0.1;
    marker.type = visualization_msgs::Marker::CYLINDER;
    vis_pub_.publish(marker);
}

void BehaviouralPathDriver::drawArrow(int id, const geometry_msgs::Pose& pose, const std::string& ns, float r, float g, float b, double live)
{
    visualization_msgs::Marker marker;
    marker.pose = pose;
    marker.ns = ns;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time();
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = id;
    marker.lifetime = ros::Duration(live);
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;
    marker.scale.x = 0.75;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.type = visualization_msgs::Marker::ARROW;
    vis_pub_.publish(marker);
}

void BehaviouralPathDriver::drawMark(int id, const geometry_msgs::Point &pos, const string &ns, float r, float g, float b)
{
    visualization_msgs::Marker marker;
    marker.pose.position = pos;
    marker.ns = ns;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time();
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = id;
    marker.lifetime = ros::Duration(3);
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.5;
    marker.type = visualization_msgs::Marker::CUBE;
    vis_pub_.publish(marker);
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

bool BehaviouralPathDriver::simpleCheckCollision(float box_width, float box_length, int dir_sign)
{
    if (dir_sign < 0) {
        // no collision check when driving backwards.
        return false;
    }


    bool collision = false;

    for (size_t i=0; i < laser_scan_.ranges.size(); ++i) {
        // project point to carthesian coordinates
        float angle = laser_scan_.angle_min + i * laser_scan_.angle_increment;
        float px = laser_scan_.ranges[i] * cos(angle);
        float py = laser_scan_.ranges[i] * sin(angle);


        /* Point p is inside the rectangle, if
             *    p.x in [-width/2, +width/2]
             * and
             *    p.y in [0, length]
             */

        if ( py >= -box_width/2 &&
             py <=  box_width/2 &&
             px >= 0 &&
             px <= box_length )
        {
            collision = true;
            break;
        }
    }

    //visualize box
    geometry_msgs::Point p1, p2, p3, p4;
    p1.y = -box_width/2;  p1.x = 0;
    p2.y = -box_width/2;  p2.x = box_length;
    p3.y = +box_width/2;  p3.x = 0;
    p4.y = +box_width/2;  p4.x = box_length;

    float r = collision ? 1 : 0;
    float g = 1 - r;
    drawLine(1, p1, p2, "laser", "collision_box", r,g,0, 3, 0.05);
    drawLine(2, p2, p4, "laser", "collision_box", r,g,0, 3, 0.05);
    drawLine(3, p1, p3, "laser", "collision_box", r,g,0, 3, 0.05);
    drawLine(4, p3, p4, "laser", "collision_box", r,g,0, 3, 0.05);

    return collision;
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


    BehaviouralPathDriver::Path& current_path = paths_[options_.path_idx];
    double distance_to_goal = current_path.back().distanceTo(current_path[options_.wp_idx]);

    if(box_length > distance_to_goal) {
        box_length = distance_to_goal + 0.2;
    }


    bool collision = MotionController::checkCollision(course, box_length, options_.collision_box_width_, enlarge_factor);


    // visualization
    if (vis_pub_.getNumSubscribers() > 0) {
        // code copied from LaserEnvironment::CheckCollision()
        // (need to recalculate this here, since LaserEnvironment can not visualize)
        //TODO: Externalize visualisation methods, to make them accessable from obstacle detector?
        float beta = current_command_.steer_front;
        float width = options_.collision_box_width_;
        float length = enlarge_factor;
        float threshold = box_length;
        // corner points of the parallelogram
        float ax,ay,bx,by,cx,cy;

        float sbeta=std::sin(beta);
        float cbeta=std::cos(beta);
        ay=width/2.0f;
        ax=0.0f;
        by=-width/2.0f;
        bx=0.0f;
        if (beta > 0) {
            ay += length*sbeta;
        } else if (beta < 0) {
            by += length*sbeta;
        }
        cy=ay+threshold*sbeta;
        cx=ax+threshold*cbeta;


        geometry_msgs::Point p1, p2, p3, p4;
        p1.y = ay;  p1.x = ax;
        p2.y = by;  p2.x = bx;
        p3.y = cy;  p3.x = cx;
        p4.y = p3.y + p2.y - p1.y;
        p4.x = p3.x + p2.x - p1.x;

        float r = collision ? 1 : 0;
        float g = 1 - r;
        drawLine(1, p1, p2, "laser", "collision_box", r,g,0, 3, 0.05);
        drawLine(2, p2, p4, "laser", "collision_box", r,g,0, 3, 0.05);
        drawLine(3, p1, p3, "laser", "collision_box", r,g,0, 3, 0.05);
        drawLine(4, p3, p4, "laser", "collision_box", r,g,0, 3, 0.05);
    }

    return collision;
}

PathFollower* BehaviouralPathDriver::getNode() const
{
    return node_;
}

void BehaviouralPathDriver::publishCommand()
{
    //ramaxx_msgs::RamaxxMsg msg = current_command_;
    geometry_msgs::Twist msg = current_command_;
    cmd_pub_.publish(msg);

    setFilteredSpeed(current_command_.velocity);
}
