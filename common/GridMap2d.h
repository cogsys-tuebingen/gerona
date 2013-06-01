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
 * @class MapArea2d
 * @brief Defines a area of the map and offers methods to access the corresponding grid cells.
 */
class MapArea2d {
public:

    virtual ~MapArea2d()
        { /* Nothing to do */ }

    /**
     * @brief Start a new iteration.
     */
    virtual void begin() = 0;

    /**
     * @brief Select next cell.
     * @return False if there are no more cells.
     */
    virtual bool next() = 0;

    /**
     * @brief Get the cell coordinates of the currently selected cell.
     * @param x x-coordinates of the cell.
     * @param y y coordinate of the cell.
     */
    virtual void getCell( int& x, int& y ) const = 0;

    /**
     * @brief Get the value of the currently selected cell.
     * @return The value.
     */
    virtual uint8_t getValue() const
        { return 0; }

    /**
     * @brief Set the value of the currently selected cell.
     * @param value The new value.
     */
    virtual void setValue( const uint8_t value )
        {}
};

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
    virtual bool point2cell( const double px, const double py, unsigned int& x, unsigned int& y ) const = 0;

    /**
     * @brief Convert cell coordinates to a point in the map coordinate system.
     * @param x x-coordinate of the cell.
     * @param y y-coordinate of the cell.
     * @param px x-coordinate of the point in the map coordinate system.
     * @param py y-coordinate of the point in the map coordinate system.
     */
    virtual void cell2point( const unsigned int x, const unsigned int y, double& px, double& py ) const = 0;

    /**
     * @brief Check if cell coordinates are valid.
     * @param x x-coordinate of the cell.
     * @param y y-coordinate of the cell.
     * @return False if the coordinates are out of range. True otherwise.
     */
    virtual bool isInMap( const int x, const int y ) const = 0;

    /**
     * @brief Check if a point int the map coordinate system lies outside
     *      of the map.
     * @param x x-coordinate of the point.
     * @param y y-coordinate of the point.
     * @return False if the coordinates are out of range. True otherwise.
     */
    virtual bool isInMap( const double x, const double y ) const = 0;

    /* Non abstract functions */

    bool isFree( const Point2d& p ) const {
        unsigned int x, y;
        point2cell( p, x, y );
        return isFree( x, y );
    }

    bool isOccupied( const Point2d& p ) const {
        unsigned int x, y;
        point2cell( p, x, y );
        return isOccupied( x, y );
    }

    bool isNoInformation( const Point2d& p ) const {
        unsigned int x, y;
        point2cell( p, x, y );
        return isNoInformation( x, y );
    }

    /**
     * @brief Convert cell coordinates to a point in the map coordinate system. (Subpixel accuracy)
     * @param x x-coordinate of the cell.
     * @param y y-coordinate of the cell.
     * @param px x-coordinate of the point in the map coordinate system.
     * @param py y-coordinate of the point in the map coordinate system.
     */
    virtual void cell2pointSubPixel( const double x, const double y, double& px, double& py ) const
        { cell2point((unsigned int) x, (unsigned int) y, px, py); }

    bool pose2cell( const Pose2d& src, unsigned int& x, unsigned int& y ) const
        { return point2cell( src.x, src.y, x, y ); }

    bool point2cell( const Point2d& p, unsigned int& x, unsigned int& y ) const
        { return point2cell( p.x, p.y, x, y ); }

    void cell2point( const unsigned int x, const unsigned int y, Point2d& p ) const
        { return cell2point( x, y, p.x, p.y ); }

    bool isInMap( const Point2d& p ) const
        { return isInMap( p.x, p.y ); }

    /**
     * @brief Set all cells in a area to the given value.
     * @param area The map area.
     * @param value The value.
     */
    virtual void setAreaValue( MapArea2d &area, const uint8_t value );

    /**
     * @brief Set all cells in a area to the value defined by that area object.
     * @param area The map area.
     */
    virtual void setAreaValue( MapArea2d &area );

    /**
     * @brief Read all cell values from a given area of the map.
     * @param area The map area.
     */
    virtual void getAreaValues( MapArea2d& area ) const;

    virtual bool isAreaFree( MapArea2d& area ) const;

    virtual bool isAreaOccupied( MapArea2d& area ) const;
};

/**
 * @class CircleArea
 * @brief Represents a circular area of a map.
 * @attention This class is mostly untested!
 */
class CircleArea : public MapArea2d {
public:

    /**
     * @brief Create an area with given center and radius.
     * @param center Center of the circle in map coordinates.
     *      Should be on the map or the area won't contain any points.
     * @param radius Radius of the circle. Should be > 0. Otherwise
     *      the area won't contain any points.
     * @param map The map. Neccessary to get the size of one cell.
     */
    CircleArea( const Point2d& center, const double radius, const GridMap2d* map );

    virtual ~CircleArea()
        { /* Nothing to do */ }

    /**
     * @brief Create the list of cells.
     * @param center Center of the circle in map coordinates.
     *      Should be on the map or the area won't contain any points.
     * @param radius Radius of the circle. Should be > 0. Otherwise
     *      the area won't contain any points.
     * @param map The map. Neccessary to get the size of one cell.
     */
    virtual void init( const Point2d& center, const double radius, const GridMap2d* map );

    /**
     * @brief Set the center point of the circular area.
     * This method is fast since it doesn't recalculate the cells in this area.
     * @param p The new center. Should be inside of the map!
     * @param map The map. Should have the same resolution as the map used to
     *      initialize the area.
     */
    virtual void setCenter( const Point2d& p, const GridMap2d *map );

    /* Inherited from MapArea2d */

    virtual void begin()
        { counter_ = 0; }

    virtual bool next() {
        if ( counter_ < cells_.size()) {
            ++counter_;
            return true;
        }
        return false;
    }

    virtual void getCell( int& x, int& y ) const {
        x = cells_[counter_].first + center_x_;
        y = cells_[counter_].second + center_y_;
    }

    virtual uint8_t getValue() const
        { return value_; }

protected:

    /**
     * @brief Adds a horizontal (in x-direction) line of cells to the cell list.
     * @param y y coordinate of the line.
     * @param x0 Maximum or minimum x coordinate.
     * @param x1 Maximum or minimum x coordinate.
     */
    void addHorizontalLine( const int y, int x0, int x1 );

    /// The cell value returned on request.
    uint8_t value_;

    /// List of all cells in this area (cell coordinates)
    std::vector<std::pair<int, int> > cells_;

    /// Current x coordinate of center point (cell coordinates)
    int center_x_;

    /// Current y coordinate of center point (cell coordinates)
    int center_y_;

    /// Points to the currently selected cell
    std::size_t counter_;
};

/**
 * @class CircleBuffer
 * @brief Used to buffer a circular area of the map.
 * @attention This class is mostly untested
 * @attention This class was designed for small areas and is not very efficient.
 */
class CircleBuffer : public CircleArea {
public:
    /**
     * @brief Create an area with given center and radius.
     * @param center Center of the circle in map coordinates.
     *      Should be on the map or the area won't contain any points.
     * @param radius Radius of the circle. Should be > 0. Otherwise
     *      the area won't contain any points.
     * @param map The map. Neccessary to get the size of one cell.
     */
    CircleBuffer( const Point2d& center, const double radius, const GridMap2d* map );

    virtual ~CircleBuffer()
        { /* Nothing to do */ }

    /**
     * @brief Create an area with given center and radius.
     * @param center Center of the circle in map coordinates.
     *      Should be on the map or the area won't contain any points.
     * @param radius Radius of the circle. Should be > 0. Otherwise
     *      the area won't contain any points.
     * @param map The map. Neccessary to get the size of one cell.
     */
    virtual void init( const Point2d& center, const double radius, const GridMap2d* map );

    /**
     * @brief Returns the previously buffered vlaue of the currently selected cell.
     * @return Previously buffered cell value.
     */
    virtual uint8_t getValue() const {
        return values_[counter_];
    }

    /**
     * @brief Buffer the value of the currently selected cell.
     * @param value Value of the currently selected cell.
     */
    virtual void setValue( const uint8_t value ) {
        values_[counter_] = value;
    }

protected:
    /// Buffered cell values.
    std::vector<uint8_t> values_;
};

/**
 * @class LineArea
 * @brief A area containing all cells on a line.
 * This class uses a Bresenham algorithm to compute the cells on a straight line.
 * @attention The width of the line is just one pixel!
 * @attention This class is mostly untested!
 */
class LineArea : public MapArea2d {
public:

    /**
     * @brief Create a new line area.
     * @attention The area won't contain any cells, if start or end is outside of the map.
     * @param start Start of the line in map coordinates.
     * @param end End of the line in map coordinates.
     * @param map The map (we need to known the origin and the resolution).
     */
    LineArea( const Point2d &start, const Point2d &end, const GridMap2d *map );

    virtual ~LineArea() { /* Nothing to do */ }

    /**
     * @brief Set start and end and compute the cells in between.
     * @attention The area won't contain any cells, if start or end is outside of the map.
     * @param start Start of the line in map coordinates.
     * @param end End of the line in map coordinates.
     * @param map The map (we need to known the origin and the resolution).
     */
    void set( const Point2d &start, const Point2d &end, const GridMap2d *map );

    /* Inherited from MapArea2d */

    virtual void begin()
        { counter_ = 0; }

    virtual bool next() {
        if ( counter_ < cells_.size()) {
            ++counter_;
            return true;
        }
        return false;
    }

    virtual void getCell( int& x, int& y ) const {
        x = cells_[counter_].first;
        y = cells_[counter_].second;
    }

    virtual uint8_t getValue() const
        { return value_; }

protected:
    /// Points to the current cell
    std::size_t counter_;

    /// All cells of this area
    std::vector<std::pair<int, int> > cells_;

    /// New cell value
    uint8_t value_;
};

} // namespace "lib_path"

#endif // GRIDMAP2D_H
