/*
 * AStar.h
 *
 *  Created on: 02.09.2010
 *      Original author: dube
 *          Modified by: marks
 */

#ifndef ASTAR_H_
#define ASTAR_H_

// C/C++
#include <vector>

// Project
#include "../common/Map.h"
#include "../common/MapMath.h"
#include "../common/Pose2d.h"
#include "OpenList.h"

namespace lib_path {

/// Represents a path in cell coordinates
typedef std::vector<waypoint_t> path_t;

/// Represents a neighbor cell
struct neighbor_t {
    /// Position of the neighbor cell
	waypoint_t position;

    /// Distance to that cell
	double distance;
};


/**
 * @class AStar
 * @brief An A* implementation.
 */
class AStar {
public:

    /**
     * @brief Create an A* object.
     *
     * @param map The map used to initialize the internal member variables.
     * @param maximumOccupancyRating ?
     */
    AStar( MapInfo* map,
           double maximumOccupancyRating = 2 );

    /**
     * @brief Relase allocated memory.
     */
	virtual ~AStar();

    /**
     * @brief Try to find a path.
     *
     * @param start Start of the path in map cell coordinates.
     * @param goal End of the path in map cell coordinates.
     *
     * @return True if a valid path was found. False otherwise.
     */
    bool planPath( const waypoint_t &start, const waypoint_t &goal );

    /**
     * @brief Return the latest path.
     *
     * @return Pointer to the latest planned path.
     *      The waypoints are given in the map cell coordinate system
     *      and the path will contain no waypoint if the latest search failed.
     */
	path_t* getLastPath();

    /**
     * @brief Set a new map.
     *
     * Note: There is no internal copy of a map. The given parameter should point to a valid map
     * as long as this object is in use. The size of the map may differ from the size of the
     * map used to initialize the object. But this will take quite some computation time.
     *
     * @param map The new map we are planning on.
     */
    void setNewMap( MapInfo* map );


private:
	waypoint_t getNearestNeighbor(const waypoint_t position);
	double getDistanceToStart(const waypoint_t position);
	double getDistanceBetweenWaypoints;
	waypoint_t addPosition2d(const waypoint_t left, const waypoint_t right);

    /**
     * @brief Try to find a path.
     *
     * @return True if there is a valid path. False otherwise.
     */
    bool search();

	bool isInMap(const waypoint_t waypoint);
	bool isFreeWay(const waypoint_t waypoint);
	double getWayRating(const waypoint_t waypoint);
	unsigned char getMapValue(const waypoint_t& waypoint);

	OpenList* mNodesList;
	static const int mNeighborCount = 8;
	neighbor_t mNeighbors[mNeighborCount];
    MapInfo* mMap;
	path_t mPath;
	waypoint_t mStart;
	waypoint_t mGoal;
    int8_t mThreshold;
    double mMaxRating;
	bool mUseRating;

};

} // Namespace "lib_path"

#endif /* ASTAR_H_ */
