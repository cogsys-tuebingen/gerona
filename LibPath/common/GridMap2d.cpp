/**
 * @file GridMap2d.cpp
 * @date Jan 2012
 * @author marks
 */


// Project
#include "GridMap2d.h"

using namespace lib_path;

///////////////////////////////////////////////////////////////////////////////
// class GridMap2d
///////////////////////////////////////////////////////////////////////////////

lib_path::GridMap2d::~GridMap2d()
{
    /* Nothing to do */
}

void GridMap2d::setAreaValue( MapArea2d &area )
{
    area.begin();
    int x, y;
    while ( area.next()) {
        area.getCell( x, y );
        if ( isInMap( x, y ))
            setValue( (unsigned int)x, (unsigned int)y, area.getValue());
    }
}

void GridMap2d::setAreaValue( MapArea2d &area, const uint8_t value )
{
    area.begin();
    int x, y;
    while ( area.next()) {
        area.getCell( x, y );
        if ( isInMap( x, y ))
            setValue( (unsigned int)x, (unsigned int)y, value );
    }
}

void GridMap2d::getAreaValues( MapArea2d &area ) const
{
    area.begin();
    int x, y;
    while ( area.next()) {
        area.getCell( x, y );
        if ( isInMap( x, y ))
            area.setValue( getValue((unsigned int)x, (unsigned int)y ));
    }
}

///////////////////////////////////////////////////////////////////////////////
// class CircleArea
///////////////////////////////////////////////////////////////////////////////

CircleArea::CircleArea( const Point2d &center,
                        const double radius,
                        const GridMap2d *map )
    : value_( 0 )
{
    init( center, radius, map );
}

void CircleArea::init( const Point2d &center,
                  const double radius,
                  const GridMap2d *map ) {
    /// @todo This function does not work correctly!!!

    cells_.clear();
    counter_ = 0;

    // The center need to be on the map and circle radius > 0
    unsigned int x0, y0;
    if ( !map->point2cell( center, x0, y0 ) || radius <= 0)
        return;

    /* Taken from http://en.wikipedia.org/wiki/Midpoint_circle_algorithm */

    unsigned int cell_radius = (int)(radius / map->getResolution());
    int f = 1 - cell_radius;
    int ddF_x = 1;
    int ddF_y = -2 * cell_radius;
    int x = 0;
    int y = cell_radius;

    cells_.push_back( std::pair<int, int>( x0, y0 + cell_radius ));
    cells_.push_back( std::pair<int, int>( x0, y0 - cell_radius ));
    addHorizontalLine( y0, x0 - cell_radius, x0 + cell_radius );

    while( x < y ) {
        if( f >= 0 ) {
            y--;
            ddF_y += 2;
            f += ddF_y;
            addHorizontalLine( y0 + y, x0 - x, x0 + x );
            addHorizontalLine( y0 - y, x0 - x, x0 + x );
            addHorizontalLine( y0 + x, x0 - y, x0 + y );
            addHorizontalLine( y0 - x, x0 - y, x0 + y );
        }
        x++;
        ddF_x += 2;
        f += ddF_x;
    }
}

void CircleArea::addHorizontalLine( const int y, int x0, int x1 )
{
    if ( x0 > x1 ) {
        int swp = x0;
        x0 = x1;
        x1 = swp;
    }

    for ( ; x0 <= x1; x0++ )
        cells_.push_back( std::pair<int, int>( x0, y ));
}

///////////////////////////////////////////////////////////////////////////////
// class CircleBuffer
///////////////////////////////////////////////////////////////////////////////

CircleBuffer::CircleBuffer( const Point2d &center,
                            const double radius,
                            const GridMap2d *map )
    : CircleArea( center, radius, map )
{ /* Nothing left to do */ }

void CircleBuffer::init( const Point2d &center,
                         const double radius,
                         const GridMap2d *map )
{
    CircleArea::init( center, radius, map );
    values_.resize( cells_.size());
    values_.assign( values_.size(), value_ );
}
