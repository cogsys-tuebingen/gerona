/**
 * @file PathDriver.cpp
 * @date March 2012
 * @author marks
 */

#include <tf/tf.h>
#include <utils/LibUtil/MathHelper.h>
#include <utils/LibUtil/Line2d.h>
#include <ramaxxbase/RamaxxMsg.h>
#include "MotionControlNode.h"

// Project
#include <motion_control/MotionResult.h>
#include "PathDriver.h"

using namespace std;
using namespace Eigen;
using namespace motion_control;

PathDriver::PathDriver( ros::Publisher& cmd_pub,MotionControlNode* node )
    : node_(node), active_( false )
{
    cmd_pub_ = cmd_pub;
    configure();
    current_speed_ = 0;

    vis_pub_ = node_->getNodeHandle().advertise<visualization_msgs::Marker>
            ("marker", 1);

    ros::NodeHandle nh_priv("~");
    nh_priv.param("front_only", front_only_, false);
    if(front_only_){
        ROS_INFO("FRONT STREER MODE");
    } else {
        ROS_INFO("DUAL STEER MODE");
    }
}

PathDriver::~PathDriver()
{
}

void PathDriver::start() {
    active_ = true;
    last_cmd_.setZero();
}

void PathDriver::stop() {
    active_ = false;
    last_cmd_.setZero();
    publishCmd( last_cmd_ );
}

int PathDriver::execute( MotionFeedback& fb, MotionResult& result ) {
    // Pending error?
    if ( pending_error_ >= 0 ) {
        int error = pending_error_;
        pending_error_ = -1; // Error is processed
        stop();
        return error;
    }

    // This might happen
    if ( !active_ ) {
        last_cmd_.setZero();
        publishCmd( last_cmd_ );
        return MotionResult::MOTION_STATUS_STOP;
    }

    // Get SLAM pose
    Vector3d slam_pose;
    geometry_msgs::Pose slampose_p;
    if ( !node_->getWorldPose( slam_pose, &slampose_p )) {
        stop();
        return MotionResult::MOTION_STATUS_SLAM_FAIL;
    }

    // Convert current waypoint into local CS
    Vector3d wp_local;
    geometry_msgs::PoseStamped wp( path_[path_idx_].pose );
    wp.header.stamp = ros::Time::now();
    if ( !node_->transformToLocal( wp, wp_local )) {
        stop();
        return MotionResult::MOTION_STATUS_SLAM_FAIL;
    }

    // Last waypoint?
    if ( (size_t)path_idx_ + 1 >= path_.size()) {
        if ( wp_local.head<2>().norm() <= goal_tolerance_ ) {
            stop();
            return MotionResult::MOTION_STATUS_SUCCESS; // Goal reached
        }
    } else {
        // Next waypoint?
        while ( (size_t)path_idx_ < path_.size() && wp_local.head<2>().norm() <= wp_tolerance_ ) {
            path_idx_++;
            wp = path_[path_idx_].pose;
            wp.header.stamp = ros::Time::now();
            if ( !node_->transformToLocal( wp, wp_local )) {
                stop();
                return MotionResult::MOTION_STATUS_SLAM_FAIL;
            }
        }
    }

    // visualize target
    drawArrow(0, wp.pose, "waypoint", 1.0, 0.7, 0.2);
    drawArrow(1, slampose_p, "waypoint", 2.0, 0.7, 1.0);

    double dir_sign = 1.0;
    if ( wp_local.x() < 0 )
        dir_sign = -1.0;

    double delta_f, delta_r;

    // Calculate target line from current to next waypoint (if there is any)
    geometry_msgs::PoseStamped next_wp;
    Line2d target_line;
    if ( (size_t)path_idx_ + 1 < path_.size()) {
        next_wp = path_[path_idx_ + 1].pose;
        next_wp.header.stamp = ros::Time::now();
        Vector3d next_wp_local;
        if ( !node_->transformToLocal( next_wp, next_wp_local )) {
            stop();
            return MotionResult::MOTION_STATUS_INTERNAL_ERROR;
        }
        target_line = Line2d( next_wp_local.head<2>(), wp_local.head<2>());
    } else {
        next_wp = wp;
        target_line.FromAngle( wp_local.head<2>(), wp_local(2) + M_PI );
    }

    // draw line
    geometry_msgs::Pose target_line_arrow;
    target_line_arrow.position = next_wp.pose.position;
    double dx = next_wp.pose.position.x - wp.pose.position.x;
    double dy = next_wp.pose.position.y - wp.pose.position.y;
    target_line_arrow.orientation = tf::createQuaternionMsgFromYaw(std::atan2(dy, dx));
    drawArrow(2, target_line_arrow, "line", 0.7, 0.2, 1.0);

    Vector2d front_pred, rear_pred;
    predictPose( dead_time_, last_cmd_(0), last_cmd_(1), getFilteredSpeed(), front_pred, rear_pred );


    // Path lost?
    if ( target_line.GetDistance( Vector2d( 0, 0 )) > 2 * wp_tolerance_ ) {
        stop();
        return MotionResult::MOTION_STATUS_PATH_LOST;
    }

    if(front_only_) {
        double e_angle = MathHelper::NormalizeAngle(tf::getYaw(wp.pose.orientation) - tf::getYaw(slampose_p.orientation));
        double e_distance = target_line.GetSignedDistance( front_pred );
        double e_f = e_distance + e_angle;


        // draw steer front
        {
        geometry_msgs::Pose steer_arrow = slampose_p;
        steer_arrow.orientation = tf::createQuaternionMsgFromYaw(tf::getYaw(steer_arrow.orientation) + e_angle);
        drawArrow(3, steer_arrow, "steer", 0.2, 1.0, 0.2);
        }
        {
        geometry_msgs::Pose steer_arrow = slampose_p;
        steer_arrow.orientation = tf::createQuaternionMsgFromYaw(tf::getYaw(steer_arrow.orientation) + e_distance);
        drawArrow(4, steer_arrow, "steer", 0.2, 0.2, 1.0);
        }
        {
        geometry_msgs::Pose steer_arrow = slampose_p;
        steer_arrow.orientation = tf::createQuaternionMsgFromYaw(tf::getYaw(steer_arrow.orientation) + e_f);
        drawArrow(5, steer_arrow, "steer", 1.0, 0.2, 0.2);
        }

        // Calculate steering angles
        if ( !mono_ctrl_.execute( e_f, delta_f))
            return MotionResult::MOTION_STATUS_MOVING; // Nothing to do

        {
        geometry_msgs::Pose steer_arrow = slampose_p;
        steer_arrow.orientation = tf::createQuaternionMsgFromYaw(tf::getYaw(steer_arrow.orientation) + delta_f);
        drawArrow(6, steer_arrow, "steer", 1.0, 1.0, 1.0);
        }

        delta_r = 0;

    } else {

        // Calculate front/rear errors
        double e_f = target_line.GetSignedDistance( front_pred );
        double e_r = target_line.GetSignedDistance( rear_pred );

        // Calculate steering angles
        if ( !dual_ctrl_.execute( e_f, e_r, delta_f, delta_r ))
            return MotionResult::MOTION_STATUS_MOVING; // Nothing to do
    }

    // Check collision and calculate speed
    double beta;
    beta = dir_sign*atan( 0.5*(tan( delta_f ) + tan( delta_r )));
    if ( dir_sign < 0 )
        beta += M_PI;
    if ( calculateSpeed( dir_sign*path_[path_idx_].speed, MathHelper::NormalizeAngle( beta ))) {
        // Collision!
        stop();
        ROS_WARN( "Collision!" );
        return MotionResult::MOTION_STATUS_COLLISION;
    }

    // Send command
    last_cmd_(0) = delta_f;
    last_cmd_(1) = delta_r;
    last_cmd_(2) = current_speed_;
    publishCmd( last_cmd_ );
    return MotionResult::MOTION_STATUS_MOVING;
}

void PathDriver::drawArrow(int id, const geometry_msgs::Pose& pose, const std::string& ns, float r, float g, float b)
{
    visualization_msgs::Marker marker;
    marker.pose = pose;
    marker.ns = ns;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time();
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = id;
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

bool PathDriver::calculateSpeed( const double request, const double beta )
{
    // We are not able to check for collisions while driving backwards
    if ( request < 0 ) {
        current_speed_ = -reverse_speed_;
        return false;
    }

    // Check for collision
    if ( checkCollision( beta, 0.35 ))
        return true;

    // Faster than requested?
    /*if ( current_speed_ > request ) {
        current_speed_ = request;
        return false;
    }*/

    // Increase speed?
    if ( checkCollision( beta, 2.0, 0.6, 2.25 ) || current_speed_ > request )
        current_speed_ -= 0.01;
    else
        current_speed_ += 0.01;
    current_speed_ = min( current_speed_, max_speed_ );
    current_speed_ = max( current_speed_, min_speed_ );
    return false;
}

void PathDriver::configure() {
//    ros::NodeHandle& nh = node_->getNodeHandle();

    ros::NodeHandle nh("~");

    // Path following/speed calculation
    nh.param( "waypoint_tolerance", wp_tolerance_, 0.20 );
    nh.param( "goal_tolerance", goal_tolerance_, 0.15 );
    nh.param( "l", l_, 0.38 );
    nh.param( "min_speed", min_speed_, 0.5 );
    nh.param( "reverse_speed", reverse_speed_, 0.5 );

    // Dual pid
    double ta, e_max, kp, ki, i_max, delta_max;
    nh.param( "dualpid/dead_time", dead_time_, 0.10 );
    nh.param( "dualpid/ta", ta, 0.03 );
    nh.param( "dualpid/e_max", e_max, 0.10 );
    nh.param( "dualpid/kp", kp, 0.4 );
    nh.param( "dualpid/ki", ki, 0.0 );
    nh.param( "dualpid/i_max", i_max, 0.0 );
    nh.param( "dualpid/delta_max", delta_max, 24.0 );
    dual_ctrl_.configure( kp, ki, i_max, M_PI*delta_max/180.0, e_max, 0.5, ta );


    nh.param( "pid/kp", kp, 0.4 );
    nh.param( "pid/ki", ki, 0.0 );
    nh.param( "pid/i_max", i_max, 0.0 );
    nh.param( "pid/delta_max", delta_max, 24.0 );

    mono_ctrl_.configure( kp, ki, i_max, M_PI*delta_max/180.0, e_max, 0.5, ta );
}


void PathDriver::setGoal( const motion_control::MotionGoal& goal ) {
    pending_error_ = -1;
    current_speed_ = getFilteredSpeed();

    // Set config
    max_speed_ = goal.v;

    // Got at least two waypoint?
    if ( goal.path.poses.size() < 2 ) {
        ROS_ERROR( "Got an invalid path with less than two poses." );
        stop();
        pending_error_ = MotionResult::MOTION_STATUS_INTERNAL_ERROR;
        return;
    }
    calculateWaypoints( goal.path );

    start();
}

void PathDriver::calculateWaypoints( const nav_msgs::Path &path )
{
    path_.clear();

    // Add first pose
    path_.push_back( Waypoint( path.poses[0], max_speed_ ));

    // For all poses execpt of the first and the last one
    Vector3d prev_wp, wp, next_wp;
    for ( size_t i = 1; i < path.poses.size() - 1; ++i ) {
        rosToEigen( path.poses[i-1].pose, prev_wp );
        rosToEigen( path.poses[i].pose, wp );
        rosToEigen( path.poses[i+1].pose, next_wp );
        path_.push_back( Waypoint( path.poses[i], calculateWaypointSpeed( prev_wp, wp, next_wp )));
    }

    // Add last waypoint
    path_.push_back( Waypoint( path.poses[path.poses.size() - 1], min_speed_ ));

    // Current waypoint is the first entry
    path_idx_ = 1;
}

double PathDriver::calculateWaypointSpeed( const Eigen::Vector3d &prev, const Eigen::Vector3d &wp, const Eigen::Vector3d &next )
{
    Vector2d q( next.head<2>() - wp.head<2>());
    Vector2d p( wp.head<2>() - prev.head<2>());

    // Too close together?
    if ( p.norm() < 0.01 || q.norm() < 0.01 ) {
        return min_speed_;
    }

    // Calculate angle
    double a = (p.dot( q ))/(p.norm()*q.norm());
    if ( a >= 1.0 ) // Avoid rounding errors
        return max_speed_;
    if ( a <= -1.0 )
        return min_speed_;
    a = 180.0*acos( a )/M_PI;

    // Calculate speed
    if ( a < 10.0 )
        return max_speed_;
    if ( a > 50.0 )
        return min_speed_;
    return min_speed_ + (a - 10.0)*(max_speed_ - min_speed_)/40.0;
}

void PathDriver::predictPose( const double dt,
                              const double deltaf,
                              const double deltar,
                              const double v,
                              Vector2d &front_pred,
                              Vector2d &rear_pred )
{
    double beta = atan(0.5*(tan(deltaf)+tan(deltar)));
    double ds = v*dt;
    double dtheta = ds*cos(beta)*(tan(deltaf)-tan(deltar))/l_;
    double thetan = dtheta;
    double yn = ds*sin(dtheta*0.5+beta*0.5);
    double xn = ds*cos(dtheta*0.5+beta*0.5);

    front_pred[0] = xn+cos(thetan)*l_/2.0;
    front_pred[1] = yn+sin(thetan)*l_/2.0;
    rear_pred[0] = xn-cos(thetan)*l_/2.0;
    rear_pred[1] = yn-sin(thetan)*l_/2.0;
}

void PathDriver::publishCmd( const Eigen::Vector3d &cmd )
{
    ramaxxbase::RamaxxMsg msg;
    msg.data.resize( 3 );
    msg.data[0].key = ramaxxbase::RamaxxMsg::CMD_STEER_FRONT_DEG;
    msg.data[1].key = ramaxxbase::RamaxxMsg::CMD_STEER_REAR_DEG;
    msg.data[2].key = ramaxxbase::RamaxxMsg::CMD_SPEED;
    msg.data[0].value = cmd(0)*180.0/M_PI;
    msg.data[1].value = cmd(1)*180.0/M_PI;
    msg.data[2].value = cmd(2);
    cmd_pub_.publish( msg );
}

void PathDriver::rosToEigen( const geometry_msgs::Pose &in, Eigen::Vector3d &out )
{
    out.x() = in.position.x;
    out.y() = in.position.y;
    out(2) = tf::getYaw( in.orientation );
}
