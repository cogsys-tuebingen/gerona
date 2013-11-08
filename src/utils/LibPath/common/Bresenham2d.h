/**
 * @file Bresenham2d.h
 * @date Jan 2012
 * @author marks
 */

#ifndef BRESENHAM2D_H
#define BRESENHAM2D_H

// Project
#include "GridMap2d.h"

namespace lib_path {

/**
 * @brief Implements the well known Bresenham algorithm to draw a line
 * The algorithm was taken from http://de.wikipedia.org/wiki/Bresenham-Algorithmus
 */
class Bresenham2d
{
public:

    /// Basic initialization
    Bresenham2d();

    /**
     * @brief Set the map, start and end
     * If start or end lies outside of the map bounds we will do
     * just nothing.
     * @param map The map
     * @param start Start of the line (map coordinates)
     * @param end End of the line (map coordinates)
     */
    void set( const GridMap2d *map, const Point2d start, const Point2d end );

    /**
     * @brief Set the map, start and end
     * If start or end lies outside of the map bounds we will do
     * just nothing.
     * @param map The map
     * @param start Start of the line (grid coordinates)
     * @param end End of the line (grid coordinates)
     */
    void setGrid(const GridMap2d *map, int start_x, int start_y, int end_x, int end_y );

    /**
     * @brief Select next cell
     * @return False if we reached the end
     */
    bool next();

    /**
     * @brief Get the map coordinates of the current cell.
     * @param p Map coordinates of the current cell
     */
    inline void point( Point2d &p ) const {
        map_->cell2point( (unsigned int)x_, (unsigned int)y_, p );
    }

    /**
     * @brief Get the coordinates of the current cell.
     * @param x x coordinate (cell coordinates)
     * @param y y coordinate (cell coordinates)
     */
    inline void coordinates( unsigned int &x, unsigned int &y ) const {
        x = x_; y = y_;
    }

    /**
     * @brief Get the value of the current cell.
     */
    inline uint8_t value() const {
        return map_->getValue( (unsigned int)x_, (unsigned int)y_ );
    }

    /**
     * @return Current cell is free?
     */
    inline bool isFree() const {
        return map_->isFree( (unsigned int)x_, (unsigned int)y_ );
    }

    /**
     * @return Current cell is occupied?
     */
    inline bool isOccupied() const {
        return map_->isOccupied( (unsigned int)x_, (unsigned int)y_ );
    }

    /**
     * @return Current cell does not contain any information?
     */
    inline bool isNoInformation() const {
        return map_->isNoInformation( (unsigned int)x_, (unsigned int)y_ );
    }

protected:

    /// The map we are working with
    const GridMap2d *map_;

    /// Start coordinates
    int x0_, y0_;

    /// End coordinates
    int x1_, y1_;

    /// Current cell
    int x_, y_;

    // Bresenham stuff
    int dx_, dy_;
    int sx_, sy_;
    int err_;
};

}

#endif // BRESENHAM2D_H
