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

// C/C++
#include <string>

// ROS
#include <ros/ros.h>
#include <tf/tf.h>
#include <ros/service_client.h>
#include <nav_msgs/GetMap.h>
#include <actionlib/server/simple_action_server.h>

// OpenCV
#include <cv.h>

// Project
#include "ExploreFrontier.h"
#include "CvMap.h"
#include <frontier_explore/ExplorationGoalsAction.h>

///////////////////////////////////////////////////////////////////////////////
// D E C L A R A T I O N S
///////////////////////////////////////////////////////////////////////////////

namespace frontier_explore {

/**
 * A node offering an frontier based exploration algorithm.
 */
class ExploreNode : public actionlib::SimpleActionServer<frontier_explore::ExplorationGoalsAction> {
public:

    /**
     * Read configuration and initialize all members.
     */
    ExploreNode( ros::NodeHandle n );

    void executeCB();

protected:

    /**
     * @brief Convert a ROS occupancy grid map to the internally used map type.
     *
     * This function will resize the map if neccessary.
     *
     * @param map The occupancy grid map.
     * @param cvmap The internally used map.
     */
    void occupancyGridToCvMap( const nav_msgs::OccupancyGrid& map, CvMap& cvmap );

    /**
     * @brief Try to get the current robot pose in map coordinates.
     *
     * @param pose The pose will be written to this parameter.
     *
     * @return False if the transform is (temporarily) not available. True otherwise.
     */
    bool getRobotPose(geometry_msgs::Pose &pose, const std::string& map_frame);

    /**
     * @brief Publish a visualization of the given explorations goals.
     *
     * Publishes an arrow marker for each goal and deletes obsolete markers. The best
     * goal will be red. All other goals are blue.
     *
     * @param goals The goals.
     */
    void publishGoalsVisu( const std::vector<geometry_msgs::Pose>& goals );

private:
    /// Map as cv image
    CvMap cvmap_;

    /// Service client to get the current map
    ros::ServiceClient map_service_client_;

    /// Used to get the robot position
    tf::TransformListener tf_;

    /// Visualization publisher
    ros::Publisher visu_pub_;

    /// Detects the frontiers
    frontier_explore::ExploreFrontier explorer_;

    /// Number of published goal markers (used to delete old markers)
    unsigned int goals_marker_count_;

    /// Map erode iterations
    int erode_;

    /// Map downsample iterations
    int downsample_;

    /// Publish visualization?
    bool visualize_;

    /// Show debug map image?
    bool show_debug_map_;
};

} // Namespace frontier_explore

#endif // _EXPLORE_NODE_H_
