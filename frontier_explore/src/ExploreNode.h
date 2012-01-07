/*
 * ExploreNode.h
 *
 * Created: Jan 2012
 * Author: Marks
 *
 */

#ifndef _EXPLORE_NODE_H_
#define _EXPLORE_NODE_H_

///////////////////////////////////////////////////////////////////////////////
// I N C L U D E S
///////////////////////////////////////////////////////////////////////////////

// ROS
#include <ros/ros.h>
#include <ros/service_client.h>
#include <nav_msgs/GetMap.h>

// Project
#include "ExploreFrontier.h"

///////////////////////////////////////////////////////////////////////////////
// D E C L A R A T I O N S
///////////////////////////////////////////////////////////////////////////////


class ExploreNode {
public:

    /**
     * Initialize the object.
     */
    ExploreNode( ros::NodeHandle n );

    bool calculateFrontiers();

private:

    /// Service client to get the current map
    ros::ServiceClient map_service_client_;

    /// Visualization publisher
    ros::Publisher  visu_pub_;

    /// Detects the frontiers
    frontier_explore::ExploreFrontier explorer_;

    /// Publish visualization?
    bool visualize_;

};


#endif // _EXPLORE_NODE_H_
