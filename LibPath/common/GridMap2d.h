/**
 * @file GridMap2d.h
 * @date Jan 2012
 * @author marks
 */

#ifndef GRIDMAP2D_H
#define GRIDMAP2D_H

// C/C++
#include <inttypes.h>

// Project
#include "Point2d.h"

namespace lib_path {

/**
 * @class GridMap
 * @brief Defines an interface for any grid based map implementations.
 */
class GridMap2d {
public:

    virtual ~GridMap2d();

    /**
     * @brief Get the value of the cell at (x,y)
     * @attention This function does not check if the given coordinates are valid.
     * @param x x-coordinate of the cell. Should be less than the width of the map.
     * @param y y-coordinate of the cell. Should be less than the height of the map.
     * @return Cell value.
     */
    virtual uint8_t getValue( const unsigned int x, const unsigned int y ) const = 0;

    /**
     * @brief Set the value of the cell at (x,y).
     * @attention This function does not check if the given cell coordinates are valid.
     * @param x x-coordinate of the cell. Should be less than the width of the map.
     * @param y y-coordinate of the cell. Should be less than the height of the map.
     * @param value The new value of the cell.
     */
    virtual void setValue( const unsigned int x, const unsigned int y, const uint8_t value ) = 0;

    /**
     * @brief Get the width of the map.
     * @return The number of cells in x-direction.
     */
    virtual unsigned int getWidth() const = 0;

    /**
     * @brief Get the height of the map.
     * @return The number of cells in y-direction.
     */
    virtual unsigned int getHeight() const = 0;

    /**
     * @brief Get the size of one cell.
     * @return The size of one cell in meter.
     */
    virtual double getResolution() const = 0;

    /**
     * @brief Get the origin of the map in the map coordinate system.
     * @return The map origin.
     */
    virtual Point2d getOrigin() const = 0;

    /**
     * @brief Set the origin of the map in the map coordinate system.
     * @param p The new origin.
     */
    virtual void setOrigin( const Point2d& p ) = 0;

    /**
     * @brief Return if a map cell is free or not.
     * @param x x-coordinate of the cell. Should be less than the width of the map.
     * @param y y-coordinate of the cell. Should be less than the height of the map.
     * @return True if the cell is free. False otherwise.
     */
    virtual bool isFree( const unsigned int x, const unsigned int y ) const = 0;

    /**
     * @brief Return if a map cell is occupied or not.
     * @param x x-coordinate of the cell. Should be less than the width of the map.
     * @param y y-coordinate of the cell. Should be less than the height of the map.
     * @return True if the cell is occupied. False otherwise.
     */
    virtual bool isOccupied( const unsigned int x, const unsigned int y ) const = 0;

    /**
     * @brief Return if we don't have any information about a map cell.
     * @param x x-coordinate of the cell. Should be less than the width of the map.
     * @param y y-coordinate of the cell. Should be less than the height of the map.
     * @return True if there is no information. False otherwise.
     */
    virtual bool isNoInformation( const unsigned int x, const unsigned int y ) const = 0;

    /**
     * @brief Convert a point in map coordinates to cell coordinates.
     * @param p The position in the map coordinate system.
     * @param x x-coordinate of the cell.
     * @param y y-coordinate of the cell.
     * @return False if the point lies outside of the map. True otherwise.
     */
    virtual bool point2Cell( const Point2d& p, int& x, int& y ) const = 0;

    /**
     * @brief Convert cell coordinates to a point in the map coordinate system.
     * @param x x-coordinate of the cell.
     * @param y y-coordinate of the cell.
     * @param p The position in the map coordinate system.
     */
    virtual void cell2point( const int x, const int y, Point2d& p ) const = 0;

    /**
     * @brief Check if cell coordinates are valid.
     * @param x x-coordinate of the cell.
     * @param y y-coordinate of the cell.
     * @return False if the coordinates are out of range. True otherwise.
     */
    virtual bool isInMap( const int x, const int y ) const = 0;

    /**
     * @brief Check a point in the map coordinate system lies outside of the map.
     * @param p The point in map coordinates.
     * @return False if the point lies outside of the map. True otherwise.
     */
    virtual bool isInMap( const Point2d& p ) const = 0;

    // To be continued...
};

} // namespace "lib_path"

#endif // GRIDMAP2D_H
