/*
 * AStar.h
 *
 *  Created on: 02.09.2010
 *      Author: dube
 */

#ifndef ASTAR_H_
#define ASTAR_H_

#include <cv.h>
#include <vector>
#include "OpenList.h"

typedef std::vector<waypoint_t> path_t;

struct neighbor_t {
	waypoint_t position;
	double distance;
};



class AStar {
public:
    AStar(IplImage* map, uchar freewayThreshold = 50,
			double maximumOccupancyRating = 2,
			int erodeIterations = 2,
			bool drawSearchInfos = false);
	virtual ~AStar();
	path_t* planPath(waypoint_t start, waypoint_t goal);
	path_t* getLastPath();
	IplImage* getMap();
	IplImage* getDebugImage();
	void setNewMap(IplImage* map);
private:
	waypoint_t getNearestNeighbor(const waypoint_t position);
	double getDistanceToStart(const waypoint_t position);
	double getDistanceBetweenWaypoints;
	waypoint_t addPosition2d(const waypoint_t left, const waypoint_t right);
	void search();
	bool isInMap(const waypoint_t waypoint);
	bool isFreeWay(const waypoint_t waypoint);
	double getWayRating(const waypoint_t waypoint);
	unsigned char getMapValue(const waypoint_t& waypoint);

	OpenList* mNodesList;
	static const int mNeighborCount = 8;
	neighbor_t mNeighbors[mNeighborCount];
	IplImage* mMap;
	IplImage* mDebugImage;
	path_t mPath;
	waypoint_t mStart;
	waypoint_t mGoal;
	uchar* mMapPointer;
	uchar mThreshold;
	double mMaxRating;
	bool mDrawSearchInfos;
	bool mUseRating;
	int mErodeIterations;

};


#endif /* ASTAR_H_ */
