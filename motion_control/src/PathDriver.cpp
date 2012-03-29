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
    : node_(node),active_( false ),
      pos_tolerance_( 0.2 )
{
    cmd_pub_ = cmd_pub;
    configure(  );
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

    // This should never happen
    if ( !active_ ) {
        ROS_WARN( "Execute called but path driver is not active. This should never happen." );
        return MotionResult::MOTION_STATUS_INTERNAL_ERROR;
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

    // Waypoint reached? Select next waypoint!
    if ( wp_local.head<2>().norm() <= pos_tolerance_ ) {
        // Last waypoint? Try to reach it!
        if (( path_idx_ + 1 ) >= (int)path_.size()) {
            /// @todo implement
        }

        // Select next waypoint
        path_idx_++;
        wp = path_[path_idx_].pose;
        wp.header.stamp = ros::Time::now();
        if ( !node_->transformToLocal( wp, wp_local )) {
            stop();
            return MotionResult::MOTION_STATUS_SLAM_FAIL;
        }
    }

    // Calculate target line
    Vector3d last_wp_local;
    wp = path_[path_idx_ - 1].pose;
    wp.header.stamp = ros::Time::now();
    if ( !node_->transformToLocal( wp, last_wp_local )) {
        stop();
        return MotionResult::MOTION_STATUS_INTERNAL_ERROR;
    }
    Line2d target_line( last_wp_local.head<2>(), wp_local.head<2>());

    // Calculate front/rear errors and steer commands
    Vector2d front_pred, rear_pred;
    predictPose( dead_time_, last_cmd_(0), last_cmd_(1), last_cmd_(2), front_pred, rear_pred );
    double e_f = target_line.GetSignedDistance( front_pred );
    double e_r = target_line.GetSignedDistance( rear_pred );
    double delta_f, delta_r;
    if ( !ctrl_.execute( e_f, e_r, delta_f, delta_r ))
        return MotionResult::MOTION_STATUS_MOVING; // Nothing to do

    // Send command
    last_cmd_(0) = delta_f;
    last_cmd_(1) = delta_r;
    last_cmd_(2) = max_speed_;
    publishCmd( last_cmd_ );
    return MotionResult::MOTION_STATUS_MOVING;
}


void PathDriver::configure() {
    ros::NodeHandle& nh=node_->getNodeHandle();
    nh.param( "path_driver/position_tolerance", pos_tolerance_, 0.2 );
    nh.param( "path_driver/l", l_, 0.36 );
}


void PathDriver::setGoal( const motion_control::MotionGoal& goal ) {
    pending_error_ = -1;

    // Set config
    max_speed_ = goal.v; /// @todo That's all?

    // Got at least one waypoint?
    if ( goal.path.poses.size() <= 0 ) {
        ROS_ERROR( "Got an empty path." );
        pending_error_ = MotionResult::MOTION_STATUS_INTERNAL_ERROR;
        return;
    }

    // Calculate waypoints
    Vector3d slam_pose;
    if ( !node_->getWorldPose( slam_pose )) {
        pending_error_ = MotionResult::MOTION_STATUS_SLAM_FAIL;
        return;
    }
    calculateWaypoints( goal.path, slam_pose );

    start();
}

void PathDriver::calculateWaypoints( const nav_msgs::Path &path, const Vector3d &slam_pose )
{
    path_.clear();

    // First waypoint is alway the current pose
    geometry_msgs::PoseStamped first_pose;
    first_pose.pose.position.x = slam_pose(0);
    first_pose.pose.position.y = slam_pose(1);
    first_pose.pose.position.z = 0;
    tf::quaternionTFToMsg( tf::createQuaternionFromYaw( slam_pose(2)),
                           first_pose.pose.orientation );
    first_pose.header = path.poses[0].header;
    path_.push_back( Waypoint( first_pose, max_speed_ ));

    // Add all other poses
    for ( size_t i = 0; i < path.poses.size(); ++i )
        path_.push_back( Waypoint( path.poses[i], max_speed_ ));

    // Current waypoint is the first entry of the actual path
    path_idx_ = 1;
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

