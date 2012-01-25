/**
 * @file SimpleGridMap2d.h
 * @date Jan 2012
 * @author marks
 */

#ifndef SIMPLEGRIDMAP2D_H
#define SIMPLEGRIDMAP2D_H

// C/C++
#include <stdint.h>
#include <vector>

// Project
#include "GridMap2d.h"
#include <stdio.h>
namespace lib_path {

/**
 * @class SimpleGridMap2d
 * @brief A simple implementation of a 2d grid map.
 */
class SimpleGridMap2d : public GridMap2d {
public:

    /**
     * @brief Create a map with the given size and resolution.
     *
     * The origin will be (0,0) and the lower threshold 50, the upper threshold 200.
     *
     * @param w Number of cells in x-direction.
     * @param h Number of cells in y-direction.
     * @param r Size of one cell in meter.
     */
    SimpleGridMap2d( const unsigned int w, const unsigned int h, const double r );

    virtual ~SimpleGridMap2d() { /* Nothing to do */ }

    /**
     * @brief Set the free cell  threshold.
     * Every cell with a value less or equal to this threshold is free.
     * @param thres The new threshold.
     */
    void setLowerThreshold( const uint8_t thres )
        { lower_thres_ = thres; }

    /**
     * @brief Set the occupied cell threshold.
     * Every cell with a value greater or equal to this threshold is occupied.
     * @param thres The new Threshold.
     */
    void setUpperThreshold( const uint8_t thres )
        { upper_thres_ = thres; }

    /**
     * @brief Set the size on one cell.
     * @param r Size of one cell in meter.
     */
    void setResolution( const double r )
        { res_ = r; }

    /**
     * @brief Set new map data.
     * @param data The new map data. Size should be >= w * h
     * @param w New map width (number of cells in x-direction)
     * @param h new map height (number of cells in y-direction)
     */
    void set( const std::vector<uint8_t>& data, const unsigned int w, const unsigned int h ) {
        width_ = w;
        height_ = h;
        data_.assign( data.begin(), data.end());
    }

    /* Inherited from GridMap2d */

    inline uint8_t getValue( const unsigned int x, const unsigned int y ) const
        { return data_[y*width_ + x]; }

    inline void setValue( const unsigned int x, const unsigned int y, const uint8_t value )
        { data_[y*width_ + x] = value; }

    inline unsigned int getWidth() const
        { return width_; }

    inline unsigned int getHeight() const
        { return height_; }

    inline double getResolution() const
        { return res_; }

    Point2d getOrigin() const
        { return origin_; }

    inline void setOrigin( const Point2d& p )
        { origin_ = p; }

    inline bool isFree( const unsigned int x, const unsigned int y ) const {
        return getValue( x, y ) <= lower_thres_;
    }

    inline bool isOccupied( const unsigned int x, const unsigned int y ) const
        { return getValue( x, y ) >= upper_thres_ ; }

    inline bool isNoInformation( const unsigned int x, const unsigned int y ) const {
        uint8_t value = getValue( x, y );
        return  value > lower_thres_ && value < upper_thres_;
    }

protected:


    /// The map data (row major order)
    std::vector<uint8_t> data_;

    /// Lower threshold. Every cell with a value equal or less to this threshold id free.
    uint8_t lower_thres_;

    /// Upper threshold. Every cell with an value greater or equal to this threshold is occupied.
    uint8_t upper_thres_;
};

} // namespace "lib_path"

#endif // SIMPLEGRIDMAP2D_H
