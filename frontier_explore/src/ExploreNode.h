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
#include <tf/tf.h>
#include <ros/service_client.h>
#include <nav_msgs/GetMap.h>

// OpenCV
#include <cv.h>

// Project
#include "ExploreFrontier.h"
#include "CvMap.h"

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

    void mapToCvMap( const nav_msgs::OccupancyGrid& map, CvMap& cvmap );

protected:

    bool getRobotPose(geometry_msgs::Pose &pose);

    void publishFrontierVisu( const std::vector<frontier_explore::Frontier>& frontiers ) const;
    void publishGoalsVisu( const std::vector<geometry_msgs::Pose>& goals );

private:
    /// Map as cv image
    CvMap cvmap_;

    /// Service client to get the current map
    ros::ServiceClient map_service_client_;

    /// Used to get the robot position
    tf::TransformListener tf_;

    /// Visualization publisher
    ros::Publisher  visu_pub_;

    /// Detects the frontiers
    frontier_explore::ExploreFrontier explorer_;

    /// Publish visualization?
    bool visualize_;

    /// Number of publishe goal markers (used to delete old markers)
    unsigned int goals_marker_count_;
};


#endif // _EXPLORE_NODE_H_
