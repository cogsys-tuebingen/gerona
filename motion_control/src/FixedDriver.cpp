/**
   (c) 2012 Karsten Bohlmann bohlmann@gmail.com

   @author Karsten Bohlmann
   @date   1/15/2012
   @file   FixedDriver.cpp

*/ 


#include "ramaxxbase/RamaxxMsg.h"
#include "Line2d.h"
#include "MathHelper.h"
#include "FixedDriver.h"

#include <cmath>

using namespace std;

FixedDriver::FixedDriver(ros::Publisher &cmd_pub, ros::NodeHandle &node)
    :cmd_pub_(cmd_pub),state_(MotionResult::MOTION_STATUS_STOP)
{
    cmd_v_ = 0;
    cmd_front_rad_ =0.0;
    cmd_rear_rad_= 0.0;
    beta_=0.0;
    dist_measure_threshold_=0.1;
}


void FixedDriver::configure(ros::NodeHandle &node)
{

}


void FixedDriver::setGoal(const motion_control::MotionGoal &goal)
{
    cmd_v_ = goal.v;
    cmd_front_rad_ = goal.deltaf;
    cmd_rear_rad_ = 0; //goal.deltar;
    beta_ = 0; //atan(0.5*(tan(cmd_front_rad_)+tan(cmd_rear_rad_)));
    getSlamPose( start_pose_ );
    last_pose_ = start_pose_;
    driven_dist_ = 0.0;
    state_ = MotionResult::MOTION_STATUS_MOVING;
    move_timer_.restart();

    generateWaypoints( goal.deltaf );
}


void FixedDriver::start()
{
}


void FixedDriver::stop()
{
    cmd_v_ = 0;
}

bool FixedDriver::update( Vector3d& slam_pose )
{
    nextWaypoint( slam_pose );

    if ( waypoints_.empty())
        return false;

    Vector3d delta = waypoints_.front() - slam_pose;
    double angle_delta = MathHelper::AngleDelta( atan2( delta.y(), delta.x()),
                                                 slam_pose(2));

    const double steer_p = 1.3;
    cmd_front_rad_ = -steer_p * angle_delta;
    //cout << "Angle d: " << angle_delta << " Front steer: " << cmd_front_rad_ << endl;

    return true;
}

void FixedDriver::nextWaypoint( Vector3d &slam_pose )
{
    const double max_wp_dist = 0.30;
    while ( !waypoints_.empty() &&
            (slam_pose.head<2>()- waypoints_.front().head<2>()).norm() < max_wp_dist )
        waypoints_.pop_front();
}

void FixedDriver::generateWaypoints( double delta_f )
{
    waypoints_.clear();

    // Distance between waypoints
    const double wp_dist = 0.25;

    // Calculate turn radius
    const double l = 0.38;
    if ( fabs( delta_f ) < 1E-3 ) {
        cout << "Fixed driver: Given front steer angle is too small. Defaults to 20 degree." << endl;
        delta_f = M_PI*20.0/180.0;
    }
    double r = l / fabs( tan( delta_f ));

    // Generate waypoints for first straight line
    Vector3d seg_start( start_pose_ );
    Vector3d seg_end;
    generateLine( seg_start, 2.0*r, wp_dist, seg_end );

    // First curve
    seg_start = seg_end;
    generateCurve( seg_start, r, wp_dist, delta_f > 0, seg_end );

    // Next straight line
    seg_start = seg_end;
    generateLine( seg_start, 2.0*r, wp_dist, seg_end );

    // Last curve
    seg_start = seg_end;
    generateCurve( seg_start, r, wp_dist, delta_f > 0, seg_end );

    /*std::list<Vector3d>::const_iterator it = waypoints_.begin();
    for ( ; it != waypoints_.end(); it++ ) {
        cout << it->x() << " " << it->y() << endl;
    }*/
}

void FixedDriver::generateCurve( const Vector3d &curve_start,
                                 const double r,
                                 const double wp_dist,
                                 const bool positive,
                                 Vector3d &curve_end )
{
    double sign = positive ? 1.0 : -1.0;
    Vector3d center;
    center.x() = curve_start.x() + r*cos( curve_start(2) + sign*0.5*M_PI );
    center.y() = curve_start.y() + r*sin( curve_start(2) + sign*0.5*M_PI );
    center(2) = MathHelper::AngleClamp( curve_start(2) - sign*0.5*M_PI );
    Vector3d wp( curve_start );

    curve_end.x() = curve_start.x() + 2.0*r*cos( curve_start(2) + sign*0.5*M_PI );
    curve_end.y() = curve_start.y() + 2.0*r*sin( curve_start(2) + sign*0.5*M_PI );
    curve_end(2) = MathHelper::AngleClamp( curve_start(2) + M_PI );

    double angle_step = sign*asin( wp_dist / r );
    double angle_steps_done = 0;
    while (  fabs( angle_steps_done ) < M_PI ) {
        center(2) = MathHelper::AngleClamp( center(2) + angle_step );
        angle_steps_done += angle_step;
        wp.x() = center.x() + r*cos( center(2));
        wp.y() = center.y() + r*sin( center(2));
        wp(2) = MathHelper::AngleClamp( center(2) + sign*0.5*M_PI );
        waypoints_.push_back( wp );
    }
}

void FixedDriver::generateLine( const Vector3d &line_start,
                                const double length,
                                const double wp_dist,
                                Vector3d &line_end )
{
    ROS_INFO( "Generationg line. Length: %f wp_dist: %f", length, wp_dist );

    Vector3d step;
    step.x() = wp_dist*cos( line_start(2));
    step.y() = wp_dist*sin( line_start(2));
    Vector3d wp( line_start );

    double dist = (line_start.head<2>()- wp.head<2>()).norm();
    while ( dist < length ) {
        wp += step;
        wp(2) = line_start(2);
        waypoints_.push_back( wp );
        dist = (line_start.head<2>()- wp.head<2>()).norm();
    }

    line_end.x() = line_start.x() + length*cos(line_start(2));
    line_end.y() = line_start.y() + length*sin(line_start(2));
    line_end(2) = line_start(2);
}

int FixedDriver::execute(MotionFeedback &fb, MotionResult &result)
{
    // check collision

    switch (state_) {
    case MotionResult::MOTION_STATUS_COLLISION:
        cmd_v_=0;
        state_=  MotionResult::MOTION_STATUS_STOP;

        break;
    case MotionResult::MOTION_STATUS_SUCCESS:
        state_=MotionResult::MOTION_STATUS_STOP;
        cmd_v_=0;
        break;
    case MotionResult::MOTION_STATUS_STOP:
        cmd_v_=0;
        result.status=MotionResult::MOTION_STATUS_STOP;
        break;
    case MotionResult::MOTION_STATUS_MOVING:
    default:
    {
        Vector3d slam_pose;
        getSlamPose( slam_pose );
        // TODO calc beta
        bool colliding=checkCollision( beta_, 0.3 );
        if ( colliding ) {
            result.status = MotionResult::MOTION_STATUS_COLLISION;
            cmd_v_ = 0.0;
        } else if ( update( slam_pose )) {
            result.status = MotionResult::MOTION_STATUS_MOVING;
        } else {
            cmd_v_ = 0;
            result.status = state_ = MotionResult::MOTION_STATUS_STOP;
        }

        double delta_dist=(slam_pose.head<2>()-last_pose_.head<2>()).norm();
        if (delta_dist>dist_measure_threshold_) {
            driven_dist_ += delta_dist;
            last_pose_ = slam_pose;
        }
        // return driven distance in feedback
        fb.dist_driven = driven_dist_;
        break;
    }
    }
    ramaxxbase::RamaxxMsg cmd;
    cmd.data.resize(3);
    cmd.data[0].key=ramaxxbase::RamaxxMsg::CMD_STEER_FRONT_DEG;
    cmd.data[1].key=ramaxxbase::RamaxxMsg::CMD_STEER_REAR_DEG;
    cmd.data[2].key=ramaxxbase::RamaxxMsg::CMD_SPEED;
    cmd.data[0].value=cmd_front_rad_*180.0/M_PI;
    cmd.data[1].value=cmd_rear_rad_*180.0/M_PI;
    cmd.data[2].value=cmd_v_;

    cmd_pub_.publish(cmd);
    return result.status;
}
