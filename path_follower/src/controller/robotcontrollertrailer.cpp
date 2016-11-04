#include <cmath>
#include <cslibs_utils/MathHelper.h>
#include <std_msgs/Float32.h>
#include <path_follower/controller/robotcontrollertrailer.h>

#include <path_msgs/FollowPathResult.h>
#include <path_follower/pathfollower.h>
#include <path_follower/utils/path_exceptions.h>
#include <path_follower/utils/obstacle_cloud.h>
#include "path_controller.h"
#include "path_simple_pid.h"
#include "path_cascade_pid.h"
#include <path_follower/utils/pose_tracker.h>

#include <pcl_ros/point_cloud.h>

using namespace std;
using namespace Eigen;

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

RobotControllerTrailer::RobotControllerTrailer(PathFollower *path_driver, ros::NodeHandle *nh):
    RobotController(path_driver),nh_(nh),
    visualizer_(Visualizer::getInstance()),
    behaviour_(ON_PATH),
    last_velocity_(0)
{



    // initialize
    std::string agv_vel_topic("/agv_vel");

    agv_vel_sub_ = nh_->subscribe<geometry_msgs::Twist> (agv_vel_topic, 10, boost::bind(&RobotControllerTrailer::updateAgvCb, this, _1));

    agv_steer_is_pub_ = nh_->advertise<std_msgs::Float32> ("/path_follower/steer_is",1);
    agv_steer_set_pub_ = nh_->advertise<std_msgs::Float32> ("/path_follower/steer_set",1);

    if (!opt_.controller_type().compare("cascade")) {
        path_ctrl_= new PathCascadePid();
    } else {
        path_ctrl_= new PathSimplePid();
    }

    obstacle_pub_ = nh->advertise<pcl::PointCloud<pcl::PointXYZRGBA>> ("path_obstacles", 1);

    double kp = 0.93;
    double ki = 0.1;
    double kd = 0.0;
    pid_psi_ = PidController<1, double>(kp, ki, kd, 0.01);

    narrow_travel_distance_ = 1.0;
}


void RobotControllerTrailer::updateAgvCb(const geometry_msgs::TwistConstPtr &vel)
{

    agv_vel_ = *vel;
}


void RobotControllerTrailer::stopMotion()
{
    cmd_.setVelocity(0);
    publishMoveCommand(cmd_);
}


void RobotControllerTrailer::reset()
{
    behaviour_ = ON_PATH;
}

RobotController::MoveCommandStatus RobotControllerTrailer::computeMoveCommand(MoveCommand *cmd)
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

    ROS_INFO_STREAM_THROTTLE(1, "state is " << behaviour_);

    if(behaviour_ != NARROW_PASSAGE) {
        // If wait_for_stop_ is set, do nothing, until the actual velocity fell below a given
        // threshold.
        // When the robot has finally stopped, go to the next subpath or quit, if goal is reached.

        if (behaviour_ == WAIT_FOR_STOP) {
            stopMotion(); //< probably not necessary to repeat this, but be on the save side.

            // do nothing until robot has realy stopped.
            geometry_msgs::Twist current_vel = pose_tracker_.getVelocity();
            if((std::abs(current_vel.linear.x) > 0.01) ||
                    (std::abs(current_vel.linear.y) > 0.01) ||
                    (std::abs(current_vel.angular.z) > 0.01)) {
                ROS_INFO_THROTTLE_NAMED(1, MODULE, "WAITING until no more motion");
                stopMotion();
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


        // check for obstacles for reactive driving
        analyzePathObstacles();

        if(behaviour_ == ON_PATH) {
            int oa_size_threshold = 2;

            int l = left.size();
            int r = right.size();
            int c = center.size();

            ROS_INFO_STREAM("obstacle count (l,c,r): " << l << "\t, " << c << "\t, " << r);

            if(c > oa_size_threshold) {
                // blocked
                stopMotion();
                return MoveCommandStatus::OKAY;
            }

            if(l > oa_size_threshold && r > oa_size_threshold) {
                // narrow passage
                behaviour_ = NARROW_PASSAGE;

                tf::Vector3 closest_l(closest_left.x, closest_left.y, 0.0);
                tf::Vector3 closest_r(closest_right.x, closest_right.y, 0.0);
                tf::Vector3 nearest_l(nearest_left.x, nearest_left.y, 0.0);
                tf::Vector3 nearest_r(nearest_right.x, nearest_right.y, 0.0);

                tf::Vector3 pivot = (closest_l + closest_r) * 0.5;

                tf::Vector3 dir_y = (nearest_r - nearest_l);
                tf::Vector3 dir_x(-dir_y.y(), dir_y.x(), 0.0);

                odom_to_base_goal_ = tf::Transform(tf::createQuaternionFromYaw(std::atan2(dir_x.y(), dir_x.x())), pivot + (narrow_travel_distance_ * dir_x).normalized());

                Line2d target_line;
                target_line = Line2d( Eigen::Vector2d(pivot.x(), pivot.y()), Eigen::Vector2d(odom_to_base_goal_.getOrigin().x(), odom_to_base_goal_.getOrigin().y()) );
                visualizer_->visualizeLine(target_line);



            } else {
                behaviour_ = ON_PATH;
            }
        }
    }

    if(behaviour_ == NARROW_PASSAGE) {
        ROS_WARN_THROTTLE(1, "driving through a narrow passage");
        last_error_pos_ = std::numeric_limits<double>::infinity();
        narrowPassage();

    } else {
        onPath();
    }

    *cmd = cmd_;

    return MoveCommandStatus::OKAY;
}

void RobotControllerTrailer::onPath()
{
    float e_dist;
    if (path_->isLastWaypoint()) {
        behaviour_ = APPROACH_SUBPATH_END;
        e_dist = getErrorApproachSubpathEnd();
    } else {
        behaviour_ = ON_PATH;
        e_dist = getErrorOnPath();
    }
    float e_angle = calculateAngleError();

    updateCommand(e_dist, e_angle);
}

void RobotControllerTrailer::narrowPassage()
{
    double trailer_length_ = 1.2;
    double min_offset_distance_ = narrow_travel_distance_ * 2;
    double min_distance_to_robot_ = 0.8;


    double min_speed_ = 0.15;
    double max_speed_ = 0.3;

    double error_okay_ = 0.06;
    double error_max_ = 0.16;

    double max_psi_ = M_PI/2;

    double trailer_angle = agv_vel_.angular.z;
    tf::Transform base_to_trailer_link_(tf::createQuaternionFromYaw(trailer_angle), tf::Vector3(std::cos(trailer_angle) * trailer_length_, std::sin(trailer_angle) * trailer_length_, 0.0));

    tf::Transform trailer_link_to_trailer_back(tf::createIdentityQuaternion(), tf::Vector3(-trailer_length_, 0, 0));
    tf::Pose base_to_trailer_back = base_to_trailer_link_ * trailer_link_to_trailer_back;

    Eigen::Vector3d pose = pose_tracker_.getRobotPose();
    tf::Transform odom_to_base_link(tf::createQuaternionFromYaw(pose(2)), tf::Vector3(pose(0), pose(1), 0.0));
    tf::Transform goal_offset(tf::createIdentityQuaternion(), tf::Vector3(narrow_travel_distance_, 0, 0));
    tf::Pose base_link_to_goal = odom_to_base_link.inverse() * odom_to_base_goal_ * goal_offset;

    // offset_moves
    double offset_distance = narrow_travel_distance_;


    tf::Pose base_link_to_base_goal = base_link_to_goal;
    while(offset_distance > -min_offset_distance_) {
        tf::Transform offset(tf::createIdentityQuaternion(), tf::Vector3(offset_distance, 0, 0));
        base_link_to_base_goal = base_link_to_goal * offset;
        if(base_link_to_base_goal.getOrigin().length() < min_distance_to_robot_) {
            break;
        }

        offset_distance -= 0.05;
    }

    tf::Pose base_link_to_trailer_goal = base_link_to_base_goal * trailer_link_to_trailer_back;

    //        tf::Transform trailer_error = trailer_back_pose.inverse() * trailer_goal_pose;
    tf::Transform trailer_back_to_trailer_goal = base_to_trailer_back.inverse() * base_link_to_trailer_goal;

    //base_link_to_base_goal.setRotation(tf::createQuaternionFromYaw(std::atan2(base_link_to_base_goal.getOrigin().y(), base_link_to_base_goal.getOrigin().x())));


    tf::Pose error;

    tf::Transform goal_to_base_link = base_link_to_goal.inverse();
    double goal_ex = std::abs(goal_to_base_link.getOrigin().x());

    tf::Transform trailer_goal_t_trailer_back = trailer_back_to_trailer_goal.inverse();

    bool position_trailer = goal_ex > trailer_length_ && std::abs(trailer_goal_t_trailer_back.getOrigin().y()) > trailer_length_ * 0.2;

    if(position_trailer) {
        error = trailer_back_to_trailer_goal;
        double error_yaw = std::atan2(error.getOrigin().y(), error.getOrigin().x());
        ROS_INFO_STREAM("trailer error yaw: " << error_yaw);
    } else {
        error = base_link_to_base_goal;
    }

    geometry_msgs::Twist twist;
    //    double error_yaw = tf::getYaw(error.getRotation());

    bool pos_good = goal_ex < error_okay_ || (goal_ex < error_max_ && goal_ex > last_error_pos_);
    last_error_pos_ = goal_ex;

    double error_yaw;
    if(goal_ex  >= min_offset_distance_) {
        error_yaw = std::atan2(error.getOrigin().y(), error.getOrigin().x());
        if(!position_trailer) {
            error_yaw *= 1.5;
        } else {
            double tey = trailer_goal_t_trailer_back.getOrigin().y();
            if(std::abs(tey) > trailer_length_ / 2.0) {
                error_yaw = 2.0 * std::atan2(trailer_back_to_trailer_goal.getOrigin().y(), trailer_back_to_trailer_goal.getOrigin().x());
            } else {
                error_yaw = 0.5 * tey;
            }
            ROS_INFO_STREAM("trailer error: " << error_yaw);
        }
    } else {
        ROS_INFO_STREAM("ey: " << goal_to_base_link.getOrigin().y());
        error_yaw = 4.0 * goal_to_base_link.getOrigin().y();
    }

    if(pos_good) {
        ROS_WARN_STREAM("done");
        behaviour_ = ON_PATH;

    } else {
        double ex = error.getOrigin().x();
        twist.linear.x = ex * 0.6;
        twist.linear.y = 0;
        double speed = std::abs(twist.linear.x);

        if(speed > max_speed_) {
            twist.linear.x *= max_speed_ / speed;
        } else if(speed < min_speed_) {
            twist.linear.x *= min_speed_ / speed;
        }

        //            double psi = std::pow(1.75 * error_yaw, 2) * (error_yaw >= 0 ? 1.0 : -1.0);
        double psi = error_yaw;


        if(std::abs(psi) > max_psi_) {
            psi = psi / std::abs(psi) * max_psi_;
        }

        pid_psi_.execute(psi, &twist.angular.z);
    }

    cmd_.setDirection(twist.angular.z );
    cmd_.setVelocity(twist.linear.x);
}


void RobotControllerTrailer::analyzePathObstacles()
{
    left.clear();
    center.clear();
    right.clear();

    double oa_max_range = 3.0;
    double oa_core_width = 0.2;
    double oa_corridor_width = 1.2;

    const SubPath& path = path_->getCurrentSubPath();
    int current_wp_idx = path_->getWaypointIndex();
    int n = path.size();

    double line_dist_l = std::numeric_limits<double>::infinity();
    double line_dist_r = std::numeric_limits<double>::infinity();
    double robot_dist_l = std::numeric_limits<double>::infinity();
    double robot_dist_r = std::numeric_limits<double>::infinity();

    Eigen::Vector3d pose = pose_tracker_.getRobotPose();
    tf::Transform odom_to_base_link(tf::createQuaternionFromYaw(pose(2)), tf::Vector3(pose(0), pose(1), 0.0));

    const auto& obstacle_cloud = path_driver_->getObstacleCloud()->cloud;
    for(auto it = obstacle_cloud->begin(); it != obstacle_cloud->end(); ++it) {
        const pcl::PointXYZ& pt = *it;
        tf::Vector3 obstacle_odom = odom_to_base_link * tf::Vector3(pt.x, pt.y, 0.0);
        Eigen::Vector2d obstacle(obstacle_odom.x(), obstacle_odom.y());

        double dist = std::hypot(pt.x, pt.y);
        if(dist < oa_max_range) {
            double min_dist = oa_corridor_width;
            int closest_wp_idx = 0;
            for(int idx = current_wp_idx; idx < n; ++idx) {
                const Waypoint& wp = path[idx];
                double pt_dist = std::hypot(wp.x - obstacle(0), wp.y - obstacle(1));
                if(pt_dist < min_dist) {
                    min_dist = pt_dist;
                    closest_wp_idx = idx;
                }
            }

            if(min_dist < oa_corridor_width) {
                const Waypoint& prev = path[std::max(0, closest_wp_idx - 1)];
                const Waypoint& next = path[std::min(n - 1, closest_wp_idx + 1)];

                Line2d line(prev, next);

                double sd = line.GetSignedDistance(obstacle);
                double abs_sd = std::abs(sd);
                if(abs_sd < oa_corridor_width / 2.0) {
                    if(abs_sd < oa_core_width / 2.0) {
                        center.push_back(pt);
                    } else {
                        if(sd > 0)  {
                            right.push_back(pt);
                            if(abs_sd < line_dist_r) {
                                line_dist_r = abs_sd;
                                closest_right = pt;
                            }

                            if(dist < robot_dist_r) {
                                robot_dist_r = dist;
                                nearest_right = pt;
                            }
                        } else {
                            left.push_back(pt);
                            if(abs_sd < line_dist_l) {
                                line_dist_l = abs_sd;
                                closest_left = pt;
                            }

                            if(dist < robot_dist_l) {
                                robot_dist_l = dist;
                                nearest_left = pt;
                            }
                        }
                    }
                }
            }
        }


    }

    if(obstacle_pub_.getNumSubscribers() > 0) {
        pcl::PointCloud<pcl::PointXYZRGBA> cloud;
        cloud.header = obstacle_cloud->header;

        double A = 20;

        pcl::PointXYZRGBA rgb;
        rgb.a = A;

        rgb.r = 0;
        rgb.g = 255;
        rgb.b = 0;
        for(const pcl::PointXYZ& pt : left) {
            rgb.x = pt.x;
            rgb.y = pt.y;
            rgb.z = pt.z;
            cloud.push_back(rgb);
        }

        rgb.a = 255;
        rgb.x = closest_left.x;
        rgb.y = closest_left.y;
        rgb.z = closest_left.z;
        cloud.push_back(rgb);
        rgb.a = A;

        rgb.r = 255;
        rgb.g = 0;
        rgb.b = 0;
        for(const pcl::PointXYZ& pt : center) {
            rgb.x = pt.x;
            rgb.y = pt.y;
            rgb.z = pt.z;
            cloud.push_back(rgb);
        }

        rgb.r = 0;
        rgb.g = 0;
        rgb.b = 255;
        for(const pcl::PointXYZ& pt : right) {
            rgb.x = pt.x;
            rgb.y = pt.y;
            rgb.z = pt.z;
            cloud.push_back(rgb);
        }

        rgb.a = 255;
        rgb.x = closest_right.x;
        rgb.y = closest_right.y;
        rgb.z = closest_right.z;
        cloud.push_back(rgb);

        obstacle_pub_.publish(cloud);
    }
}

void RobotControllerTrailer::publishMoveCommand(const MoveCommand &cmd) const
{
    geometry_msgs::Twist msg;
    if (cmd.isValid()) {
        msg.linear.x = cmd.getVelocity();
        msg.angular.z = cmd.getDirectionAngle();
        last_velocity_ = cmd.getVelocity();
    }
    // else (=not valid): dont modify msg --> all set to zero, that is robot will stop.
    // ROS_INFO_STREAM_THROTTLE(0.3,"desired steer angle "<<msg.angular.z*180.0/M_PI);
    cmd_pub_.publish(msg);
}


void RobotControllerTrailer::selectWaypoint()
{
    double tolerance = path_driver_->getOptions().wp_tolerance();

    // increase tolerance, when driving backwards
    if(getDirSign() < 0) {
        tolerance *= 1;
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

    const Waypoint& wp = path_->getCurrentWaypoint();
    if (wp.actuator_cmds_.size()>=2) {
        steer_des_fwd_ = wp.actuator_cmds_[0];
        steer_des_bwd_ = wp.actuator_cmds_[1];
    } else {
        steer_des_fwd_ = steer_des_bwd_ =0.0;
    }
    wp_map.pose = path_->getCurrentWaypoint();
    wp_map.header.stamp = ros::Time::now();

    if (!pose_tracker_.transformToLocal(wp_map, next_wp_local_)) {
        throw EmergencyBreakException("cannot transform next waypoint",
                                      path_msgs::FollowPathResult::RESULT_STATUS_TF_FAIL);
    }

}



namespace {
double angleDifference(double s1, double s2) {
    double dy = std::sin(s1) - std::sin(s2);
    double dx = std::cos(s1) - std::cos(s2);
    return std::atan2(dy, dx);
}
}


double RobotControllerTrailer::calculateAngleError()
{

    geometry_msgs::Pose waypoint   = path_->getCurrentWaypoint();
    geometry_msgs::Pose robot_pose = pose_tracker_.getRobotPoseMsg();
    double trailer_angle;

    /*
    bool status=getTrailerAngle("base_link","/trailer_link",trailer_angle);
    if (!status) {
        trailer_angle=0.0;
    }*/
    trailer_angle =  agv_vel_.angular.z;
    //    return MathHelper::AngleClamp(e);
    return MathHelper::AngleClamp(tf::getYaw(waypoint.orientation) - tf::getYaw(robot_pose.orientation) + trailer_angle);
}


float RobotControllerTrailer::getErrorOnPath()
{
    /* The error is the sum of the orientation angle error and the distance to the line that
     * goes through the next waypoints.
     * The velocity is set to the maximum.
     */

    // Calculate target line from current to next waypoint (if there is any)
    float error = calculateLineError();

    return error;
}

float RobotControllerTrailer::getErrorApproachSubpathEnd()
{
    /* The error is the sum of the orientation angle error and the sideways distance to the
     * waypoint (that is the distance on the y-axis in the robot frame)
     * The velocity is decreasing with the distance to the waypoint.
     */

    // Calculate target line from current to next waypoint (if there is any)
    double error = calculateSidewaysDistanceError();

    if (visualizer_->hasSubscriber()) {
        visualizer_->drawCircle(2, ((geometry_msgs::Pose) path_->getCurrentWaypoint()).position,
                                0.5, getFixedFrame(), "turning point", 1, 1, 1);

    }

    return error;
}




void RobotControllerTrailer::updateCommand(float dist_error, float angle_error)
{
    // draw steer front
    if (visualizer_->hasSubscriber()) {
        visualizer_->drawSteeringArrow(path_driver_->getFixedFrameId(), 1, pose_tracker_.getRobotPoseMsg(), angle_error, 0.2, 1.0, 0.2);
        visualizer_->drawSteeringArrow(path_driver_->getFixedFrameId(), 2, pose_tracker_.getRobotPoseMsg(), dist_error, 0.2, 0.2, 1.0);
    }


    double v = fabs(agv_vel_.linear.x);

    // call PID controller for steering.

    std::vector<double> u(1), target_u(1);

    bool do_control = path_ctrl_->execute(dist_error,angle_error,v*dir_sign_,target_u,u);
    if (!do_control) {
        return; // Nothing to do
    }
    float u_val = u[0];

    if(std::isnan(u_val)) {
        ROS_WARN_STREAM("u_val is NaN!");
        cmd_.setVelocity(0);
        return;
    }

    visualizer_->drawSteeringArrow(path_driver_->getFixedFrameId(), 14, pose_tracker_.getRobotPoseMsg(), u_val, 0.0, 1.0, 1.0);

    float steer = dir_sign_* std::max(-opt_.max_steer(), std::min(u_val, opt_.max_steer()));

    //  ROS_INFO_STREAM_THROTTLE(0.3,"max steer "<<opt_.max_steer()*180.0/M_PI<<" direction "<<dir_sign_);
    //  ROS_INFO_STREAM_THROTTLE(0.3,"updateval steer angle "<<steer*180.0/M_PI<< " u "<<u_val*180.0/M_PI);

    //    ROS_DEBUG_STREAM_NAMED(MODULE, "direction = " << dir_sign_ << ", steer = " << steer);

    //    ROS_WARN_STREAM_NAMED(MODULE, "==========");

    //    ROS_WARN_STREAM_NAMED(MODULE, "dist_error = " << dist_error << ", angle_error = " << angle_error << ", v = " << v << ", dir_sign_ = " << dir_sign_);
    //    ROS_WARN_STREAM_NAMED(MODULE, "u_val = " << u_val << ", v = " << v);
    //    ROS_WARN_STREAM_NAMED(MODULE, "direction = " << dir_sign_ << ", steer = " << steer);

    // Control velocity
    float velocity = controlVelocity(steer);

    double steer_des, steer_tol;
    if (dir_sign_>0) {
        steer_des = steer_des_fwd_;
        steer_tol = opt_.fwd_cap_steer_deg()*M_PI/180.0;
    } else {
        steer_des = steer_des_bwd_;
        steer_tol = opt_.bwd_cap_steer_deg()*M_PI/180.0;
    }
    if (steer>steer_des && fabs(steer-steer_des)>steer_tol) {
        steer=steer_des+steer_tol;
    }
    if (steer<steer_des && fabs(steer-steer_des)>steer_tol) {
        steer=steer_des-steer_tol;
    }


    cmd_.setDirection(steer);
    cmd_.setVelocity(dir_sign_ * velocity);
    std_msgs::Float32 msg;
    msg.data = agv_vel_.angular.z;
    agv_steer_is_pub_.publish(msg);

    msg.data=dir_sign_ * steer;
    agv_steer_set_pub_.publish(msg);

}

float RobotControllerTrailer::controlVelocity(float steer_angle) const
{
    PathFollowerParameters path_driver_opt = path_driver_->getOptions();
    float velocity = velocity_;

    if(fabsf(steer_angle) > path_driver_opt.steer_slow_threshold()) {
        //ROS_INFO_STREAM_THROTTLE_NAMED(2, MODULE, "slowing down");
        velocity *= 0.75;
    }
    //***todo rewrite

    // Reduce maximal velocity, when driving backwards.
    /*  if(dir_sign_ < 0) {
        velocity = min(velocity, 0.4f * path_driver_opt.max_velocity());
    }*/

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

double RobotControllerTrailer::distanceToWaypoint(const Waypoint &wp) const
{
    Eigen::Vector3d pose = pose_tracker_.getRobotPose();
    return std::hypot(pose(0) - wp.x, pose(1) - wp.y);
}

void RobotControllerTrailer::predictPose(Vector2d &front_pred, Vector2d &rear_pred) const
{



    double trailer_angle = agv_vel_.angular.z;
    /* bool status=getTrailerAngle("/trailer_link","base_link",trailer_angle);
    if (!status) {
        trailer_angle = 0.0;
        // ***todo error handling
    }*/
    double v_current = agv_vel_.linear.x;
    double dt = opt_.dead_time();


    if (fabsf(trailer_angle)<0.1*M_PI/180.0) {
        // agv drives approx straight
        front_pred[0] = v_current*dt;
        front_pred[1] = 0;
        rear_pred[0] = front_pred[0] - opt_.l();
        rear_pred[1] = 0;

    } else {
        // radius desired turn circle


        double l = opt_.l();
        if(dir_sign_ < 0) {
            l *= 5;
        }

        double r_f = fabs(l/sin(trailer_angle));
        double r_h = fabs(l/tan(trailer_angle));
        double ds = v_current*dt; // sign of velocity matters
        double dtheta = ds/r_f;
        front_pred[0] = r_f*sin(dtheta);
        rear_pred[0]= -r_h*sin(fabs(trailer_angle)-dtheta);
        if (trailer_angle>=0) {
            front_pred[1] = r_f*(1.0-cos(dtheta));
            rear_pred[1] = r_f-r_h*cos(fabs(trailer_angle)-dtheta);
        } else {
            front_pred[1] = r_f*(cos(dtheta)-1.0);
            rear_pred[1] = -r_f+r_h*cos(fabs(trailer_angle)-dtheta);
        }

    }



    //  ROS_DEBUG_STREAM_NAMED(MODULE, "predict pose. front: " << front_pred << ", rear: " << rear_pred);
}

double RobotControllerTrailer::calculateLineError() const
{
    Vector3d next_wp_local;
    Vector3d followup_next_wp_local;

    bool has_line_points = false;
    geometry_msgs::PoseStamped tmp_pt;
    tmp_pt.header.stamp = ros::Time::now();

    // search forwards from current, if not at the end
    if(path_->getWaypointIndex() + 1 < path_->getCurrentSubPath().size()){
        next_wp_local = next_wp_local_;

        int wpi = path_->getWaypointIndex() + 1;
        while(!has_line_points) {
            Vector3d next;
            tmp_pt.pose = path_->getWaypoint(wpi);
            if (!pose_tracker_.transformToLocal( tmp_pt, next)) {
                throw EmergencyBreakException("Cannot transform next waypoint",
                                              path_msgs::FollowPathResult::RESULT_STATUS_TF_FAIL);
            }

            if((next - next_wp_local).head<2>().norm() > 1e-2) {
                followup_next_wp_local = next;
                has_line_points = true;
            } else {
                ++wpi;
                if(wpi >= (int) path_->getCurrentSubPath().size()) {
                    break;
                }
            }
        }
    }

    // search backwards from goal
    if(!has_line_points) {
        if(path_->getWaypointIndex() + 1 == path_->getCurrentSubPath().size()) {
            tmp_pt.pose = path_->getWaypoint(path_->getWaypointIndex());
            if (!pose_tracker_.transformToLocal( tmp_pt, followup_next_wp_local)) {
                throw EmergencyBreakException("Cannot transform next waypoint",
                                              path_msgs::FollowPathResult::RESULT_STATUS_TF_FAIL);
            }

            int wpi = path_->getWaypointIndex() - 1;
            while(!has_line_points) {
                Vector3d next;
                tmp_pt.pose = path_->getWaypoint(wpi);
                if (!pose_tracker_.transformToLocal( tmp_pt, next)) {
                    throw EmergencyBreakException("Cannot transform next waypoint",
                                                  path_msgs::FollowPathResult::RESULT_STATUS_TF_FAIL);
                }

                if((next - followup_next_wp_local).head<2>().norm() > 1e-2) {
                    next_wp_local = next;
                    has_line_points = true;
                } else {
                    --wpi;
                    if(wpi < 0) {
                        break;
                    }
                }
            }
        }
    }


    if((followup_next_wp_local - next_wp_local).head<2>().norm() < 1e-2 || !has_line_points) {
        ROS_ERROR_STREAM("cannot determine line");
        throw EmergencyBreakException("cannot determine line",
                                      path_msgs::FollowPathResult::RESULT_STATUS_INTERNAL_ERROR);
    }

    Line2d target_line;
    //    ROS_INFO_STREAM("line : " << next_wp_local << " -> " << followup_next_wp_local << " :  " << (followup_next_wp_local - next_wp_local).norm());
    target_line = Line2d( next_wp_local.head<2>(), followup_next_wp_local.head<2>());
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
        visualizeCarrot(main_carrot, 100, 1.0,0.0,1.0);
        visualizeCarrot(alt_carrot, 101, 0.0,1.0,1.0);
    }

    if(dir_sign_ >= 0) {
        return -target_line.GetSignedDistance(main_carrot) - 0.25 * target_line.GetSignedDistance(alt_carrot);
    } else {
        return -1.25 * target_line.GetSignedDistance(main_carrot) - 0.3 * target_line.GetSignedDistance(alt_carrot);
    }
}

double RobotControllerTrailer::calculateSidewaysDistanceError() const
{
    return calculateLineError();

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
        visualizeCarrot(main_carrot, 100, 1.0,1.0,0.1);
        visualizeCarrot(alt_carrot, 101, 0.1,1.0,1.0);
    }


    // TODO: code from approach pose
    double dist_on_y_axis = next_wp_local_[1] - main_carrot[1];
    if(std::abs(dist_on_y_axis) < tolerance) {
        return 0;
    } else {
        return dist_on_y_axis;
    }
}

void RobotControllerTrailer::visualizeCarrot(const Vector2d &carrot,
                                             int id, float r, float g, float b) const
{
    geometry_msgs::PoseStamped carrot_local;
    carrot_local.pose.position.x = carrot[0];
    carrot_local.pose.position.y = carrot[1];

    carrot_local.pose.orientation = tf::createQuaternionMsgFromYaw(0);
    geometry_msgs::PoseStamped carrot_map;
    if (pose_tracker_.transformToGlobal(carrot_local, carrot_map)) {
        visualizer_->drawCircle(id, carrot_map.pose.position, 0.2, getFixedFrame(),"pred", r,g,b,1,5);
    }
}


bool RobotControllerTrailer::getTrailerAngle( double& angle) const
{
    angle = agv_vel_.angular.z;
    return true;
}


void RobotControllerTrailer::setPath(Path::Ptr path)
{
    RobotController::setPath(path);

    path->precomputeSteerCommands(this);


}

void RobotControllerTrailer::precomputeSteerCommand(Waypoint &wp_now,  Waypoint &wp_next)
{
    double dist = wp_now.distanceTo(wp_next);
    // we don't know yet if the path is driven fwd orr bwd, thus both directions are computed
    wp_next.actuator_cmds_.resize(2);
    double dtheta =wp_next.orientation-wp_now.orientation;
    dtheta = MathHelper::NormalizeAngle(dtheta);

    if (fabs(dtheta)<0.5*M_PI/180.0) {
        // movement straight
        wp_next.actuator_cmds_[0]=wp_next.actuator_cmds_[1]=0.0;
    } else {
        double l=opt_.l(); // wheelbase
        double radius = dist/(2*sin(fabs(dtheta)/2.0));
        if (l>fabs(radius)) {
            std::cout << "radius too small "<< radius << " dtheta is "<<dtheta*180.0/M_PI << std::endl;
            wp_next.actuator_cmds_[0] = 0.0;
            wp_next.actuator_cmds_[1] = 0.0;

        } else {
            double steer_fwd = asin(l/radius);
            double steer_bwd = atan(l/radius);
            if (dtheta<0) {
                steer_fwd = -1.0*steer_fwd;
            } else {
                steer_bwd = -1.0*steer_bwd;
            }
            wp_next.actuator_cmds_[0] = steer_fwd;
            wp_next.actuator_cmds_[1] = steer_bwd;
        }
    }
    /*   std::cout << "wp steer fwd "<<wp_next.actuator_cmds_[0]*180.0/M_PI
              << " steer bwd "<<wp_next.actuator_cmds_[1]*180.0/M_PI << std::endl;
*/
}
