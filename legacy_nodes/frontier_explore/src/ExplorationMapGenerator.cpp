/*
 * ExplorationMapGenerator.cpp
 *
 * Created: April 2012
 * Author: Marks
 *
 */

///////////////////////////////////////////////////////////////////////////////
// I N C L U D E S
///////////////////////////////////////////////////////////////////////////////

// Project
#include "ExplorationMapGenerator.h"

///////////////////////////////////////////////////////////////////////////////
// I M P L E M E N T A T I O N
///////////////////////////////////////////////////////////////////////////////

ExplorationMapGenerator::ExplorationMapGenerator()
    : map( 10, 10, 1.0 ) // Dummy initialization
{
}

void ExplorationMapGenerator::update(
        const lib_path::GridMap2d &ground_map,
        const lib_path::GridMap2d &ceiling_map )
{
    // Check map size and resolution and resize map if neccessary
    if ( ground_map.getWidth() != map.getWidth() || ground_map.getHeight() != map.getHeight()
         || ground_map.getResolution() != map.getResolution()) {
        map = lib_path::SimpleGridMap2d( ground_map.getWidth(), ground_map.getHeight(), ground_map.getResolution());
        map.setLowerThreshold( 20 );
        map.setUpperThreshold( 80 );
    }
    map.setOrigin( ground_map.getOrigin());

    // Set all cells to no information
    map.set( 50 );

    // For all cells in the ground map
    unsigned int w = ground_map.getWidth();
    unsigned int h = ground_map.getHeight();
    lib_path::Point2d p;
    for ( unsigned int x = 0; x < w; ++x ) {
        for ( unsigned int y = 0; y < h; ++y ) {
            ground_map.cell2point( x, y, p );
            if ( ground_map.isOccupied( x, y ))
                map.setValue( x, y, 100 );
            else if ( ground_map.isFree( x, y ) && ceiling_map.isInMap( p ) && ceiling_map.isOccupied( p ))
                map.setValue( x, y, 0 );
        }
    }
}
