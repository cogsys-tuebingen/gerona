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

bool GridMap2d::isAreaFree( MapArea2d &area ) const
{
    area.begin();
    int x, y;
    while ( area.next()) {
        area.getCell( x, y );
        if ( isInMap( x, y ) && !isFree( x, y ))
            return false;
    }
    return true;
}

bool GridMap2d::isAreaOccupied( MapArea2d &area ) const
{
    area.begin();
    int x, y;
    while ( area.next()) {
        area.getCell( x, y );
        if ( isInMap( x, y ) && isOccupied( x, y ))
            return true;
    }
    return false;
}

///////////////////////////////////////////////////////////////////////////////
// class CircleArea
///////////////////////////////////////////////////////////////////////////////

CircleArea::CircleArea( const Point2d &center,
                        const double radius,
                        const GridMap2d *map )
    : value_( 0 ),
      center_x_( 0 ),
      center_y_( 0 )
{
    init( center, radius, map );
}

void CircleArea::setCenter( const Point2d &p, const GridMap2d *map )
{
    unsigned int x, y;
    if ( map->point2cell( p, x, y )) {
        center_x_ = (int)x;
        center_y_ = (int)y;
    }
}

void CircleArea::init( const Point2d &center,
                  const double radius,
                  const GridMap2d *map ) {

    cells_.clear();
    counter_ = 0;

    // The center need to be on the map and circle radius > 0
    unsigned int x0, y0;
    if ( !map->point2cell( center, x0, y0 ) || radius <= 0)
        return;
    setCenter( center, map );

    // Radius <= cell size?
    if ( radius <= map->getResolution()) {
        cells_.push_back( std::pair<int, int>( 0, 0 ));
        return;
    }

    /* Taken from http://en.wikipedia.org/wiki/Midpoint_circle_algorithm */

    /// @todo This function is not perfect (sometimes too much cells). Replace it if there is enough time

    int cell_radius = (int)(radius / map->getResolution());
    int f = 1 - cell_radius;
    int ddF_x = 1;
    int ddF_y = -2 * cell_radius;
    int x = 0;
    int y = cell_radius;

    addHorizontalLine( 0, - cell_radius, 0 + cell_radius );

    while( x < cell_radius ) {
        if( f >= 0 ) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;
        addHorizontalLine( x, -y, y );
        addHorizontalLine( -x, -y, y );
    }

    /*for ( std::size_t i = 0; i < cells_.size(); ++i ) {
        std::cout << "(" << cells_[i].first << ", " << cells_[i].second << ")" << endl;
    }*/
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
{
    init( center, radius, map );
}

void CircleBuffer::init( const Point2d &center,
                         const double radius,
                         const GridMap2d *map )
{
    CircleArea::init( center, radius, map );
    values_.resize( cells_.size());
    values_.assign( values_.size(), value_ );
}
