/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

/*
 * File: ExploreFrontier.h
 * Created: Jan 2012
 *
 * The original code was taken from ROS package "exploration/explore".
 *
 * Original author: Duhadway
 * Modified by: Marks
 *
 */

#ifndef EXPLORE_FRONTIER_H_
#define EXPLORE_FRONTIER_H_

// ROS
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <Eigen/Core>

// Workspace
#include <utils/LibPath/common/GridMap2d.h>
#include <utils/LibPath/a_star/AStar.h>

namespace frontier_explore {

/// Represents one point of a frontier. Internally used
struct FrontierPoint {

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// Cell x coordinate
    unsigned int x;

    /// Cell y coordinate
    unsigned int y;

    /// Direction
    Eigen::Vector3d dir;

    /// Contructor that initializes the cell indices and the direction
    FrontierPoint( unsigned int cell_x, unsigned int cell_y, Eigen::Vector3d d )
        : x( cell_x ), y( cell_y ), dir( d ) {}

};

/// Represents a frontier
struct Frontier {

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// Position and orientation (x, y, theta)
    Eigen::Vector3d pose;

    /// Number of cells
    unsigned int size;

    /// Default contructor
    Frontier() : pose( 0, 0, 0 ), size(0) {}

    /// Contructor that initializes the pose and the number of cells
    Frontier( const Eigen::Vector3d& p, const unsigned int& s ) { pose = p; size = s; }
};

/// Represents a weighted frontier
struct WeightedFrontier {

    /// The frontier
    Frontier frontier;

    /// Cost of the frontier
    double cost;

    /// Cost defaults to 1E9
    WeightedFrontier() : frontier(), cost( 1e9 ) {}

    bool operator<( const WeightedFrontier& o ) const { return cost < o.cost; }
};

/**
 * @class ExploreFrontier
 * @brief A class that will identify frontiers in a partially explored map
 */
class ExploreFrontier {

public:
    ExploreFrontier();
    virtual ~ExploreFrontier();

    /**
     * @brief Returns all frontiers.
     *
     * @param map The map to search for frontiers
     * @param frontiers Will be filled with current frontiers
     * @return True if at least one frontier was found
     */
    virtual bool getFrontiers( const lib_path::GridMap2d &map, std::vector<Frontier>& frontiers );

    /**
     * @brief Returns a list of frontiers, sorted by estimated cost to visit each frontier.
     *
     * @param explore_map The map to search for frontiers
     * @param ground_map Used to check if a goal is reachable or not
     * @param robot_pose The current position of the robot
     * @param goals Will be filled with sorted list of current goals
     * @return True if at least one frontier was found
     */
    virtual bool getExplorationGoals(
            lib_path::GridMap2d &explore_map,
            lib_path::GridMap2d &ground_map,
            const Eigen::Vector3d& robot_pose,
            std::vector<WeightedFrontier>& goals );

    /// Set minimum frontier length in meter
    virtual void setMinFrontierLength( const double min_length ) { min_frontier_length_ = min_length; }

    /// Set frontier distance gain in frontier cost calculation
    virtual void setPathLengthGain( const double gain ) { path_length_gain_ = gain; }

    /// Set orientation change gain in frontier cost calculation
    virtual void setOrientationChangeGain( const double gain ) { orientation_change_gain_ = gain; }

    /// Set frontier length gain in frontier cost calculation
    virtual void setFrontierLengthGain( const double gain ) { frontier_length_gain_ = gain; }

protected:

    /**
    * @brief Finds frontiers and populates frontiers_
    * @param map The map to search for frontiers
    */
    virtual void findFrontiers( const lib_path::GridMap2d& map );

    /**
    * @brief Calculates cost to explore frontier
    * @param frontier to evaluate
    */
    virtual bool getFrontierCost( const lib_path::GridMap2d& map,
                                  const Frontier& frontier,
                                  const Eigen::Vector3d& robot_pose,
                                  double &cost );

    /// All frontiers
    std::vector<Frontier> frontiers_;

private:
    /// Minimum frontier length
    double min_frontier_length_;

    /// Gain of distance to the frontier in cost calculation
    double path_length_gain_;

    /// Gain of orientation change in cost calculation
    double orientation_change_gain_;

    /// Gain of frontier length in cost calculation
    double frontier_length_gain_;

    /// Planner used to determine if a frontier is reachable and to calculate the distance to it
    lib_path::AStar* planner_;
};

}

#endif /* EXPLORE_FRONTIER_H_ */
