/**
 * @file brain_node.h
 * @date July 2012
 * @author marks
 */

#ifndef _BRAIN_NODE_H_
#define _BRAIN_NODE_H_

// C/C++
#include <vector>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Point.h>

namespace multiple_persons_brain {

class BrainNode {
public:

    /**
     * @brief Read config an initialize object.
     */
    BrainNode();

    /**
     * @brief Run the node and process messages etc.
     */
    void spin();

    /**
     * @brief Check persons position and set new targets
     *      if neccessary.
     */
    void updateCallback( const ros::TimerEvent& e );

protected:

    /// Update timer
    ros::Timer update_timer_;

    /// List of all target points
    std::vector<geometry_msgs::Point> targets_;
};

} // namespace

#endif // _BRAIN_NODE_H_
