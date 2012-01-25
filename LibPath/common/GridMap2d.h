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
     * The result is not necessarily a whole number.
     * @param px x-coordinate of the position in the map coordinate system.
     * @param py y-coordinate of the position in the map coordinate system.
     * @param x x-coordinate of the cell.
     * @param y y-coordinate of the cell.
     * @return False if the point lies outside of the map. True otherwise.
     */
    bool point2cell( const double px, const double py, unsigned int& x, unsigned int& y ) const
    {
      x = (unsigned int)((px - origin_.x)/res_);
      y = (unsigned int)((py - origin_.y)/res_);

      if ( !isInMap( (int)x, (int)y ))
            return false;
        return true;

    }
    /**
     * @brief Convert cell coordinates to a point in the map coordinate system.
     * @param x x-coordinate of the cell.
     * @param y y-coordinate of the cell.
     * @param px x-coordinate of the point in the map coordinate system.
     * @param py y-coordinate of the point in the map coordinate system.
     */
    void cell2point( const unsigned int x, const unsigned int y, double& px, double& py ) const
    {
         px = res_*(double)(x+0.5) + origin_.x;
         py = res_*(double)(y+0.5) + origin_.y;
         //printf("cp x=%d y=%d px=%f py=%f res=%f\n",x,y,px,py,res_);

    }



    inline bool isInMap( const int x, const int y ) const
        { return !(x < 0 || y < 0 || x > (int)width_ || y > (int)height_); }

    inline bool isInMap( const double x, const double y ) const
        { return (x - origin_.x)/res_ < width_ && (y - origin_.y)/res_ < height_; }



    /**
     * @brief Check if cell coordinates are valid.
     * @param x x-coordinate of the cell.
     * @param y y-coordinate of the cell.
     * @return False if the coordinates are out of range. True otherwise.
     */

    /**
     * @brief Check if a point int the map coordinate system lies outside
     *      of the map.
     * @param x x-coordinate of the point.
     * @param y y-coordinate of the point.
     * @return False if the coordinates are out of range. True otherwise.
     */

    // To be continued...

    /* Non abstract functions */

    virtual inline bool isFree( const Point2d& p ) const {
        unsigned int x, y;
        point2cell( p, x, y );
        return isFree( x, y );
    }

    virtual inline bool isOccupied( const Point2d& p ) const {
        unsigned int x, y;
        point2cell( p, x, y );
        return isOccupied( x, y );
    }

    virtual inline bool isNoInformation( const Point2d& p ) const {
        unsigned int x, y;
        point2cell( p, x, y );
        return isNoInformation( x, y );
    }

    virtual inline bool pose2cell( const Pose2d& src, unsigned int& x, unsigned int& y ) const
        { return point2cell( src.x, src.y, x, y ); }

    virtual inline bool point2cell( const Point2d& p, unsigned int& x, unsigned int& y ) const
        { return point2cell( p.x, p.y, x, y ); }

    virtual inline void cell2point( const unsigned int x, const unsigned int y, Point2d& p )
        { return cell2point( x, y, p.x, p.y ); }

    virtual inline bool isInMap( const Point2d& p ) const
        { return isInMap( p.x, p.y ); }

protected:
    /// Number of cells in x-direction
    unsigned int width_;

    /// Number of cells in y-direction
    unsigned int height_;

    /// Size of one cell in meter
    double res_;

    /// Origin of the map
    Point2d origin_;

};

} // namespace "lib_path"

#endif // GRIDMAP2D_H
