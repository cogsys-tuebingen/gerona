#ifndef STUCKTIMEOUT_H
#define STUCKTIMEOUT_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

/**
 * @brief Timeout to abort path execution if robot gets stuck.
 *
 * The timeout mechanism is very simple: with a constant frequency, the change in position of the robot is checked.
 * If the distance to the position in the last period is too small, the is_stuck-flag is set to true, otherwise the
 * robot seems to move, so the flag is set to false.
 *
 * Note that the time set in the ctor is therefore *not* to be interpreted as the time after which the path execution
 * aborts when the robot is not moving, but real time-until-abort can be a bit longer but is less 2*time (if the robot
 * gets stuck immediately after the last check).
 */
class StuckTimeout
{
public:

    StuckTimeout(ros::NodeHandle &nh, float time, float tolerance):
        pos_tolerance_(tolerance),
        is_stuck_(false)
    {
        timer_ = nh.createTimer(ros::Duration(time), &StuckTimeout::timerCB, this);
        odom_sub_ = nh.subscribe<nav_msgs::Odometry>("/odom", 1, &StuckTimeout::odometryCB, this);
    }

    void odometryCB(const nav_msgs::OdometryConstPtr &msg)
    {
        position_ = msg->pose.pose.position;
    }

    bool isStuck() const
    {
        return is_stuck_;
    }

    void start()
    {
        timer_start_position_ = position_;
        is_stuck_ = false;

        timer_.start();
    }

    void stop()
    {
        timer_.stop();
    }


private:
    ros::Subscriber odom_sub_;
    ros::Timer timer_;
    float pos_tolerance_;

    geometry_msgs::Point position_;
    geometry_msgs::Point timer_start_position_;
    bool is_stuck_;


    //! Callback of the stuck timer.
    /** Aborts path execution if the robot is not moving for a full period  */
    void timerCB(const ros::TimerEvent &event)
    {
        // calc distance
        float dx = timer_start_position_.x - position_.x;
        float dy = timer_start_position_.y - position_.y;
        float dist = sqrt(dx*dx + dy*dy);

        is_stuck_ = (dist < pos_tolerance_);

        timer_start_position_ = position_;
    }
};

#endif // STUCKTIMEOUT_H
