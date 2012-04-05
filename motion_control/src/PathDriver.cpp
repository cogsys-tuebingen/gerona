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
        return MotionResult::MOTION_STATUS_STOP;
    }

    // Get SLAM pose
    Vector3d slam_pose;
    if ( !node_->getWorldPose( slam_pose )) {
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

    // Calculate target line from current to next waypoint (if there is any)
    Line2d target_line;
    if ( (size_t)path_idx_ + 1 < path_.size()) {
        wp = path_[path_idx_ + 1].pose;
        wp.header.stamp = ros::Time::now();
        Vector3d next_wp_local;
        if ( !node_->transformToLocal( wp, next_wp_local )) {
            stop();
            return MotionResult::MOTION_STATUS_INTERNAL_ERROR;
        }
        target_line = Line2d( next_wp_local.head<2>(), wp_local.head<2>());
    } else {
        target_line.FromAngle( wp_local.head<2>(), wp_local(2) + M_PI );
    }

    // Calculate front/rear errors and steer commands
    double dir_sign = 1.0;
    if ( wp_local.x() < 0 )
        dir_sign = -1.0;
    Vector2d front_pred, rear_pred;
    predictPose( dead_time_, last_cmd_(0), last_cmd_(1), dir_sign*path_[path_idx_].speed, front_pred, rear_pred );
    double e_f = target_line.GetSignedDistance( front_pred );
    double e_r = target_line.GetSignedDistance( rear_pred );
    double delta_f, delta_r;
    if ( !ctrl_.execute( e_f, e_r, delta_f, delta_r ))
        return MotionResult::MOTION_STATUS_MOVING; // Nothing to do

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
    if ( current_speed_ > request ) {
        current_speed_ = request;
        return false;
    }

    // Increase speed?
    if ( checkCollision( beta, 1.5, 0.35, 1.75 ))
        current_speed_ -= 0.1;
    else
        current_speed_ += 0.025;
    current_speed_ = min( current_speed_, max_speed_ );
    current_speed_ = max( current_speed_, min_speed_ );
    return false;
}

void PathDriver::configure() {
    ros::NodeHandle& nh = node_->getNodeHandle();

    // Path following/speed calculation
    nh.param( "path_driver/waypoint_tolerance", wp_tolerance_, 0.15 );
    nh.param( "path_driver/goal_tolerance", goal_tolerance_, 0.1 );
    nh.param( "path_driver/l", l_, 0.38 );
    nh.param( "path_driver/min_speed", min_speed_, 0.4 );
    nh.param( "path_driver/reverse_speed", reverse_speed_, 0.4 );

    // Dual pid
    double ta, e_max, kp, delta_max;
    nh.param( "path_driver/dualpid/dead_time", dead_time_, 0.1 );
    nh.param( "path_driver/dualpid/ta", ta, 0.05 );
    nh.param( "path_driver/dualpid/e_max", e_max, 0.1 );
    nh.param( "path_driver/dualpid/kp", kp, 0.6 );
    nh.param( "path_driver/dualpid/delta_max", delta_max, 22.0 );
    ctrl_.configure( kp, M_PI*delta_max/180.0, e_max, 0.5, ta );
}


void PathDriver::setGoal( const motion_control::MotionGoal& goal ) {
    pending_error_ = -1;

    // Set config
    max_speed_ = goal.v; /// @todo That's all?

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
    path_idx_ = 0;
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
    if ( a < 5.0 )
        return max_speed_;
    if ( a > 50.0 )
        return min_speed_;
    return min_speed_ + (a - 5.0)*(max_speed_ - min_speed_)/45.0;
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
