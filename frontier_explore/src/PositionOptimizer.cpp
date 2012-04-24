
///////////////////////////////////////////////////////////////////////////////
// I N C L U D E S
///////////////////////////////////////////////////////////////////////////////

// Workspace
#include <CircleFinder.h>

// Project
#include "PositionOptimizer.h"

///////////////////////////////////////////////////////////////////////////////
// I M P L E M E N T A T I O N
///////////////////////////////////////////////////////////////////////////////

namespace frontier_explore {

PositionOptimizer::PositionOptimizer( const double map_size )
    : map_( NULL ), map_size_( map_size )
{
    initializeMap( 0.1 );
}

PositionOptimizer::~PositionOptimizer()
{
    if ( map_ != NULL )
        delete map_;
}

bool PositionOptimizer::optimize(const Eigen::Vector2d &pos,
                                 const lib_path::GridMap2d *map,
                                 Eigen::Vector2d &new_pos )
{
    // New map resolution?
    initializeMap( map->getResolution());

    // Map origin
    double half_size = 0.5*map_->getHeight()*map_->getResolution();
    map_->setOrigin( lib_path::Point2d( pos.x() - half_size, pos.y() - half_size));

    // Generate freespace
    calculateFreespace( map, pos );

    // Create list of obstacles
    unsigned int w = map_->getWidth();
    unsigned int h = map_->getHeight();
    std::vector<Point> obstacles;
    Point p;
    for ( unsigned int x = 0; x < w; ++x ) {
        for ( unsigned int y = 0; y < h; ++y ) {
            if ( !map_->isFree( x, y )) {
                p.x = x; p.y = y;
                obstacles.push_back( p );
            }
        }
    }

    // Run freespace finder
    CircleFinder circle_finder( 0, (int)w, 0, (int)h, (int)( 0.4 / map_->getResolution()));
    circle_finder.run( obstacles );

    // Success?
    if ( circle_finder.valid()) {
        p = circle_finder.get_center();
        map_->cell2point( (unsigned int)p.x, (unsigned int)p.y, new_pos.x(), new_pos.y());
        return true;
    }

    return false;
}

void PositionOptimizer::calculateFreespace( const lib_path::GridMap2d *ext_map,
                                            const Eigen::Vector2d pos )
{
    unsigned int size = map_->getHeight(); // Width and height are equal
    lib_path::Point2d start( pos.x(), pos.y()), end;

    // For all border cells in x direction
    for ( unsigned int x = 0; x < size; ++x ) {
        map_->cell2point( x, 0, end.x, end.y );
        calculateFreeline( ext_map, start, end );
        map_->cell2point( x, size - 1, end.x, end.y );
        calculateFreeline( ext_map, start, end );
    }

    // For all border cells in y direction (except the ones in the corners)
    for ( unsigned int y = 1; y < size - 1; ++y ) {
        map_->cell2point( 0, y, end.x, end.y );
        calculateFreeline( ext_map, start, end );
        map_->cell2point( size - 1, y, end.x, end.y );
        calculateFreeline( ext_map, start, end );
    }
}

void PositionOptimizer::calculateFreeline( const lib_path::GridMap2d *ext_map,
                                           const lib_path::Point2d start,
                                           const lib_path::Point2d end )
{
    bres_.set( ext_map, start, end );
    lib_path::Point2d p;
    unsigned int x, y;
    while ( bres_.next()) { // We will skip the origin here
        if ( bres_.isFree()) {
            bres_.point( p );
            if ( !map_->point2cell( p.x, p.y, x, y ))
                break; /// @todo Neccessary?
            map_->setValue( x, y, 0 );
        } else {
            break;
        }
    }
}

void PositionOptimizer::initializeMap( const double res )
{
    // Create new map?
    if ( map_ == NULL || map_->getResolution() != res ) {
        int size = map_size_ / res;
        if ( map_ != NULL )
            delete map_;
        // Width and heigth should be always equal
        map_ = new lib_path::SimpleGridMap2d( size, size, res );
    }

    // Set all cells occupied
    map_->set( 255 );
}

} // namespace
