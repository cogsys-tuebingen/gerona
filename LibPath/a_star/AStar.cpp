/*
 * AStar.cpp
 *
 *      Created on: 02.09.2010
 * Original author: dube
 *     Modified by: marks
 */

// C/C++
#include <iostream>
#include <cmath>

// Project
#include "AStar.h"

using namespace std;
using namespace lib_path;

AStar::AStar( MapInfo *map, double maximumOccupancyRating ) {
	mMap = NULL;
	mNodesList = NULL;
    setNewMap( map );

	mMaxRating = maximumOccupancyRating;
	mUseRating = (mMaxRating > 1);

	mNeighbors[0].position.x = -1; mNeighbors[0].position.y = -1; mNeighbors[0].distance = sqrt(2);
	mNeighbors[1].position.x = 0; mNeighbors[1].position.y = -1; mNeighbors[1].distance = 1;
	mNeighbors[2].position.x = 1; mNeighbors[2].position.y = -1; mNeighbors[2].distance = sqrt(2);
	mNeighbors[3].position.x = -1; mNeighbors[3].position.y = 0; mNeighbors[3].distance = 1;
	mNeighbors[4].position.x = 1; mNeighbors[4].position.y = 0; mNeighbors[4].distance = 1;
	mNeighbors[5].position.x = -1; mNeighbors[5].position.y = 1; mNeighbors[5].distance = sqrt(2);
	mNeighbors[6].position.x = 0; mNeighbors[6].position.y = 1; mNeighbors[6].distance = 1;
	mNeighbors[7].position.x = 1; mNeighbors[7].position.y = 1; mNeighbors[7].distance = sqrt(2);
}

AStar::~AStar() {
    if ( mNodesList != NULL )
		delete mNodesList;
}

void AStar::setNewMap( MapInfo *map ) {
    // Check if there is an existing map
    if ( mMap != NULL && ( mMap->width != map->width || mMap->height != map->height )) {
        cout << "AStar: Warning: Set map with new size. This takes some computation time." << endl;
        delete mNodesList;
        mNodesList = NULL;
    }
    mMap = map;

    // Create node list only if necessary
    if ( mNodesList == NULL )
        mNodesList = new OpenList( mMap->width, mMap->height );
}

bool AStar::planPath( const waypoint_t &start, const waypoint_t &goal ) {
    // Path contains no waypoints if the search fails
    mPath.clear();

    // Check start/goal cell indices
    if ( !isInMap( start )) {
        cout << ("AStar: Start is out of map.") << endl;
        return false;
    }
    if ( !isInMap( goal )) {
        cout << ("AStar: Goal is out of map.") << endl;
        return false;
    }

    // Run
    mStart = start;
    mGoal = goal;
    return search();
}

bool AStar::search() {
    // Prepare
	mNodesList->prepareNewSearch();
    mNodesList->addNode( mGoal, 0, getDistanceToStart(mGoal), NULL );
	Node* parent = mNodesList->getFirstNode();

    // Until we found a path or until we visited all cells
    while( parent->getPosition().x != mStart.x || parent->getPosition().y != mStart.y ) {
        for ( int i = 0; i < mNeighborCount; ++i ) {
            waypoint_t pose = addPosition2d( parent->getPosition(), mNeighbors[i].position );
            if ( !isFreeWay( pose ))
				continue;
            /*if ( mUseRating )
				mNodesList->addNode(pose,
						parent->getDistance() + (mNeighbors[i].distance * getWayRating(pose)),
						getDistanceToStart(pose),
						parent);
            else*/
            mNodesList->addNode(pose,
                    parent->getDistance() + (mNeighbors[i].distance),
                    getDistanceToStart(pose),
                    parent );
 		}
		parent = mNodesList->getFirstNode();
        // No more cells. No path!
        if ( parent == NULL )
            return false;
	}

    // Go from start backwards to the end of the path...
	Node* node = parent;
    while ( node != NULL ) {
        this->mPath.push_back( node->getPosition());
		node = node->getParent();
	}

    return true;
}

path_t* AStar::getLastPath() {
	return &this->mPath;
}

bool AStar::isInMap( const waypoint_t waypoint ) {
    return !(waypoint.x < 0 || waypoint.y < 0 || waypoint.x >= mMap->width || waypoint.y >= mMap->height);
}

bool AStar::isFreeWay( const waypoint_t waypoint ) {
    if (!isInMap( waypoint ))
		return false;

    int8_t value = mMap->getValue( waypoint.x, waypoint.y );
    return value <= mMap->threshold_max && value >= mMap->threshold_min;
}

double AStar::getWayRating( const waypoint_t waypoint ) {
    /*int8_t value = mMap->getValue( waypoint.x, waypoint.y );
    if (!isFreeWay( waypoint ))
		return mMaxRating;

	uchar range = 255 - mThreshold;
	double rating = (1 - (value - mThreshold) / (double)range) * mMaxRating + 1;//((value - mThreshold) / (double)range + 1) * mMaxRating;
    return rating;*/
    return 1.0;
}

double AStar::getDistanceToStart(waypoint_t position) {
	return sqrt( pow(position.x - mStart.x, 2) +
			pow(position.y - mStart.y, 2));
}

waypoint_t AStar::addPosition2d(const waypoint_t left, const waypoint_t right) {
	waypoint_t result;
	result.x = left.x + right.x;
	result.y = left.y + right.y;
	return result;
}
