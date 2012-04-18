#ifndef FRONTIERFINDER_H
#define FRONTIERFINDER_H

///////////////////////////////////////////////////////////////////////////////
// I N C L U D E S
///////////////////////////////////////////////////////////////////////////////

// C/C++
#include <vector>

// ROS
#include <Eigen/Core>

// Workspace
#include <utils/LibPath/common/GridMap2d.h>

///////////////////////////////////////////////////////////////////////////////
// D E C L A R A T I O N S
///////////////////////////////////////////////////////////////////////////////

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

    /// Position and orientation (x, y, theta) in map coordinates
    Eigen::Vector3d pose;

    /// Number of cells
    unsigned int size;

    /// Default contructor
    Frontier() : pose( 0, 0, 0 ), size(0) {}

    /// Contructor that initializes the pose and the number of cells
    Frontier( const Eigen::Vector3d& p, const unsigned int& s ) { pose = p; size = s; }
};

/**
 * @brief Offers methods to find frontiers in a map.
 */
class FrontierFinder
{
public:

    /**
     * @brief Returns all frontiers.
     *
     * @param map The map to search for frontiers
     * @param min_frontier_length Minimum side length of a frontiers bounding box.
     * Smaller frontiers will be neglected.
     * @param frontiers Will be filled with current frontiers
     * @return True if at least one frontier was found
     */
    static bool getFrontiers( const lib_path::GridMap2d *map,
                              const double min_frontier_length,
                              std::vector<Frontier>& frontiers );
};

} // namespace

#endif // FRONTIERFINDER_H
