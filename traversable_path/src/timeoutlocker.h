#ifndef TIMEOUTLOCKER_H
#define TIMEOUTLOCKER_H

#include <ros/ros.h>

/**
 * @brief A simple, universal locker class with unlock timeout.
 *
 * This class has an internal locked-stat which can be set to "locked" or "unlocked". If locked, and not unlocked
 * manually, it will be unlocked automaticly after the specified timeout.
 *
 * @author Felix Widmaier
 * @version $Id$
 */
class TimeoutLocker
{
public:
    /**
     * @brief Constructor.
     * @param timeout The duration after which the locker will unlock automaticly.
     */
    TimeoutLocker(ros::Duration timeout);

    /**
     * @brief Set state to "locked".
     * Sets the state to "locked" and starts the unlock timer at the same time, which will automaticly unlock the goal
     * after the timeout specified in the constructor.
     */
    void lock();

    //! Set state to "unlocked".
    void unlock();

    //! True if state is "locked", false if "unlocked".
    bool isLocked();

private:
    //! ros node handle. Required for the timer.
    ros::NodeHandle node_handle_;
    //! Locked state.
    bool locked_;
    //! Timer for automatic unlock.
    ros::Timer unlock_timer_;

    //! Unlock timer callback (sets state to unlocked).
    void timerCallback(const ros::TimerEvent& event);
};

#endif // TIMEOUTLOCKER_H
