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
    map.setLowerThreshold( 20 );
    map.setUpperThreshold( 80 );
}

void ExplorationMapGenerator::update(
        const lib_path::GridMap2d &ground_map,
        const lib_path::GridMap2d &ceiling_map )
{
    // Check map size and resolution and resize map if neccessary
    if ( ground_map.getWidth() != map.getWidth() || ground_map.getHeight() != map.getHeight()
         || ground_map.getResolution() != map.getResolution()) {
        map = lib_path::SimpleGridMap2d( ground_map.getWidth(), ground_map.getHeight(), ground_map.getResolution());
    }

    // For all cells in the ground map
    unsigned int w = ground_map.getWidth();
    unsigned int h = ground_map.getHeight();
    lib_path::Point2d p;
    unsigned int q, r;
    for ( unsigned int x = 0; x < w; ++x ) {
        for ( unsigned int y = 0; y < h; ++y ) {
            ground_map.cell2point( x, y, p );
            if ( ground_map.isFree( x, y ) && ceiling_map.point2cell( p, q, r )
                 && ( ceiling_map.isFree( p ) || ceiling_map.isNoInformation( p ))) {
                map.setValue( x, y, 50 ); // Set cell to unknown
            } else if ( ground_map.isNoInformation( x, y )){
                map.setValue( x, y, 50 );
            } else {
                map.setValue( x, y, ground_map.getValue( x, y ));
            }
        }
    }
}
