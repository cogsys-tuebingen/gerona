/**
 * @file Bresenham2d.cpp
 * @date Jan 2012
 * @author marks
 */

// Project
#include "Bresenham2d.h"

namespace lib_path {

Bresenham2d::Bresenham2d()
{
    x0_ = y0_ = x_ = y_ = x1_ = y1_ = 0;
}

void Bresenham2d::set( const GridMap2d *map, const Point2d start, const Point2d end )
{
    // Start or end outside of map?
    if ( !map->isInMap( start ) || !map->isInMap( end )) {
        x0_ = y0_ = x_ = y_ = x1_ = y1_ = 0;
        return;
    }

    // Start/end in cell coordinates
    map_ = map;
    /// @todo Fix this signed/unsigned shit
    unsigned int x, y;
    map_->point2cell( start, x, y );
    x0_ = x; y0_ = y;
    map_->point2cell( end, x, y );
    x1_ = x; y1_ = y;

    // Initialize Bresenham variables
    dx_ =  abs( x1_ - x0_ ), sx_ = x0_ < x1_ ? 1 : -1;
    dy_ = -abs( y1_ - y0_ ), sy_ = y0_ < y1_ ? 1 : -1;
    err_ = dx_ + dy_;

    // Initial cell
    x_ = x0_;
    y_ = y0_;
}

bool Bresenham2d::next()
{
    if ( x_ == x1_ && y_ == y1_ )
        return false;

    int e2 = 2*err_;
    if ( e2 > dy_ ) { err_ += dy_; x_ += sx_; }
    if ( e2 < dx_ ) { err_ += dx_; y_ += sy_; }

    return true;
}

} // namespace


