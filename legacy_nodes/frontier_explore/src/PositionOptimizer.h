#ifndef POSITIONOPTIMIZER_H
#define POSITIONOPTIMIZER_H

///////////////////////////////////////////////////////////////////////////////
// I N C L U D E S
///////////////////////////////////////////////////////////////////////////////

// ROS
#include <Eigen/Core>

// Workspace
#include <utils/LibPath/common/SimpleGridMap2d.h>
#include <utils/LibPath/common/Bresenham2d.h>

///////////////////////////////////////////////////////////////////////////////
// D E C L A R A T I O N S
///////////////////////////////////////////////////////////////////////////////

namespace frontier_explore {

class PositionOptimizer
{
public:
    PositionOptimizer( const double map_size );

    virtual ~PositionOptimizer();

    bool optimize( const Eigen::Vector2d &pos, const lib_path::GridMap2d* map, Eigen::Vector2d &new_pos );

private:

    /**
     * @brief Adapt the map resolution and set all cells to occupied.
     * @param res Map resolution
     */
    void initializeMap( const double res );

    /**
     * @brief Calculate free cells.
     * Use a Bresenham approach to calculate all cells that are free and
     * visible from the given position.
     * @param ext_map The map
     * @param pos The position (in map coordinates)
     */
    void calculateFreespace( const lib_path::GridMap2d* ext_map, const Eigen::Vector2d pos );

    /**
     * @brief Calculate free cells on a line.
     * @param ext_map The map
     * @param start Start of the line in map coordinates
     * @param end End of line in map coordinates
     */
    void calculateFreeline( const lib_path::GridMap2d* ext_map, const lib_path::Point2d start, const lib_path::Point2d end );

    /// Map used for the freespace finder
    lib_path::SimpleGridMap2d *map_;

    /// Used to calculate the free space
    lib_path::Bresenham2d bres_;

    /// Sied length of the map in meter
    double map_size_;
};

} // namespace

#endif // POSITIONOPTIMIZER_H
