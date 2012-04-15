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
#include <nav_msgs/OccupancyGrid.h>
#include <actionlib/server/simple_action_server.h>
#include <Eigen/Core>

// Workspace
#include <utils/LibRosUtil/OccupancyGridWrapper.h>

// Project
#include "ExploreFrontier.h"
#include "ExplorationMapGenerator.h"
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

    /**
     * @brief Called if there is a new goal to process.
     */
    void executeCB();

    /**
     * @brief New ground map callback.
     * @param map New map.
     */
    void groundMapCB( const nav_msgs::OccupancyGridConstPtr& map );

    /**
     * @brief New ceiling map callback.
     * @param map New map.
     */
    void ceilingMapCB( const nav_msgs::OccupancyGridConstPtr& map );

protected:

    /**
     * @brief Try to get the current robot pose in map coordinates.
     *
     * @param pose The pose will be written to this parameter.
     *
     * @return False if the transform is (temporarily) not available. True otherwise.
     */
    bool getRobotPose( Eigen::Vector3d& pose, const std::string& map_frame);

    /**
     * @brief Publish a visualization of the given explorations goals.
     *
     * Publishes an arrow marker for each goal and deletes obsolete markers. The best
     * goal will be red. All other goals are blue.
     *
     * @param goals The goals.
     */
    void publishGoalsVisu( const std::vector<WeightedFrontier>& goals );

private:

    /// Copy of the current ground map
    nav_msgs::OccupancyGrid ground_map_data_;

    /// Copy of the current ceiling map
    nav_msgs::OccupancyGrid ceiling_map_data_;

    /// Used to get the robot position
    tf::TransformListener tf_;

    /// Visualization publisher
    ros::Publisher visu_pub_;

    /// Listenes for new ground maps
    ros::Subscriber ground_map_subs_;

    /// Listens for new ceiling maps (3D exploration)
    ros::Subscriber ceiling_map_subs_;

    /// Detects the frontiers
    frontier_explore::ExploreFrontier explorer_;

    /// Number of published goal markers (used to delete old markers)
    unsigned int goals_marker_count_;

    /// Publish visualization?
    bool visualize_;

    /// Use a map of the ceiling (3D exploration)?
    bool use_ceiling_map_;

    /// Flag if we got at least one ground map
    bool got_ground_map_;

    /// Flag if we got at least one ceiling map
    bool got_ceiling_map_;

    /// Used to combine the ground and ceiling maps
    ExplorationMapGenerator map_gen_;
};

} // Namespace frontier_explore

#endif // _EXPLORE_NODE_H_
