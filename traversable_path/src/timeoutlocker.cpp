#include "timeoutlocker.h"

TimeoutLocker::TimeoutLocker(ros::Duration timeout):
    locked_(false)
{
    unlock_timer_ = node_handle_.createTimer(timeout, &TimeoutLocker::timerCallback, this, true, false);
}

void TimeoutLocker::lock()
{
    ROS_DEBUG("Lock.");
    locked_ = true;
    unlock_timer_.start();
}

void TimeoutLocker::unlock()
{
    ROS_DEBUG("Unlock.");
    locked_ = false;
    unlock_timer_.stop();
}

bool TimeoutLocker::isLocked()
{
    return locked_;
}

void TimeoutLocker::timerCallback(const ros::TimerEvent &event)
{
    ROS_DEBUG("Unlock after timeout.");
    locked_ = false;
}
