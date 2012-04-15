/*
 * ExplorationMapGenerator.h
 *
 * Created: April 2012
 * Author: Marks
 *
 */

#ifndef EXPLORATIONMAPGENERATOR_H
#define EXPLORATIONMAPGENERATOR_H

///////////////////////////////////////////////////////////////////////////////
// I N C L U D E S
///////////////////////////////////////////////////////////////////////////////

// Workspace
#include <utils/LibPath/common/SimpleGridMap2d.h>

///////////////////////////////////////////////////////////////////////////////
// D E C L A R A T I O N S
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Combines a ceiling and a ground map to something we call "exploration map".
 * Used for indoor 3D exploration. A map cell is explored, if the ground cell is free and the
 * corresponding ceiling cell is occupied (scanned).
 */
class ExplorationMapGenerator
{
public:

    /**
     * @brief Contructor. Does nothing.
     * @attention Be sure to update the map at least once before using it.
     */
    ExplorationMapGenerator();

    /**
     * @brief Calculate the combined exploration map.
     * @param ground_map The ground map.
     * @param ceiling_map The ceiling map.
     */
    void update( const lib_path::GridMap2d &ground_map, const lib_path::GridMap2d &ceiling_map );

    /// The combined exploration map. Same size and resolution as the ground map!
    lib_path::SimpleGridMap2d map;

};

#endif // EXPLORATIONMAPGENERATOR_H
