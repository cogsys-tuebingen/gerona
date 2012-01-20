/*
 * AStar.cpp
 *
 *  Created on: 02.09.2010
 *      Author: dube
 */

#include <iostream>
#include <cmath>
#include <stddef.h>
#include <highgui.h>
#include <a_star/AStar.h>

using namespace std;

AStar::AStar(IplImage* map, uchar freewayThreshold,
		double maximumOccupancyRating, int erodeIterations,
		bool drawSearchInfos) {
	mMap = NULL;
	mNodesList = NULL;
	mDrawSearchInfos = drawSearchInfos;
	mErodeIterations = erodeIterations;
	setNewMap(map);

	mThreshold = freewayThreshold;
	mMaxRating = maximumOccupancyRating;
	mUseRating = (mMaxRating > 1);
	mDebugImage = NULL;

	mNeighbors[0].position.x = -1; mNeighbors[0].position.y = -1; mNeighbors[0].distance = sqrt(2);
	mNeighbors[1].position.x = 0; mNeighbors[1].position.y = -1; mNeighbors[1].distance = 1;
	mNeighbors[2].position.x = 1; mNeighbors[2].position.y = -1; mNeighbors[2].distance = sqrt(2);
	mNeighbors[3].position.x = -1; mNeighbors[3].position.y = 0; mNeighbors[3].distance = 1;
	mNeighbors[4].position.x = 1; mNeighbors[4].position.y = 0; mNeighbors[4].distance = 1;
	mNeighbors[5].position.x = -1; mNeighbors[5].position.y = 1; mNeighbors[5].distance = sqrt(2);
	mNeighbors[6].position.x = 0; mNeighbors[6].position.y = 1; mNeighbors[6].distance = 1;
	mNeighbors[7].position.x = 1; mNeighbors[7].position.y = 1; mNeighbors[7].distance = sqrt(2);
	/*mNeighbors[0].position.x = 0; mNeighbors[0].position.y = -1; mNeighbors[0].distance = 1;
	mNeighbors[1].position.x = -1; mNeighbors[1].position.y = 0; mNeighbors[1].distance = 1;
	mNeighbors[2].position.x = 1; mNeighbors[2].position.y = 0; mNeighbors[2].distance = 1;
	mNeighbors[3].position.x = 0; mNeighbors[3].position.y = 1; mNeighbors[3].distance = 1;*/

}

AStar::~AStar() {
	cvReleaseImage(&mMap);
	if (mDebugImage != NULL)
		cvReleaseImage(&mDebugImage);
	if (mNodesList != NULL)
		delete mNodesList;
}

void AStar::setNewMap(IplImage* map) {
	if (mMap != NULL) {
		if (mMap->widthStep != map->widthStep || mMap->height != map->height) {
			cout << "AStar: Warning: Set map with new size. This takes some computation time." << endl;
			delete mNodesList;
			mNodesList = NULL;
		}
		cvReleaseImage(&mMap);
	}
	if (map->nChannels != 1) {
		cout << "AStar.cpp: Map image has more than one channel. Object is not properly set up." << endl;
		return;
	}
	mMap = cvCreateImage(cvSize(map->width, map->height), map->depth, 1);
	//cvCvtColor(map, mMap, CV_BGR2GRAY);
	//cvCopyImage(map, mMap);
	cvErode(map, mMap, NULL, mErodeIterations);
	mMapPointer = (uchar*) mMap->imageData;

	if (mNodesList == NULL)
		mNodesList = new OpenList(mMap->widthStep, mMap->height);
}

path_t* AStar::planPath(waypoint_t start, waypoint_t goal){
	mStart = start;
	mGoal = goal;
	mPath.clear();


	if (mDrawSearchInfos) {
		if (mDebugImage != NULL)
			cvReleaseImage(&mDebugImage);
		mDebugImage = cvCreateImage(
				cvSize(mMap->width, mMap->height),
				mMap->depth, 3);
		cvCvtColor(mMap, mDebugImage, CV_GRAY2RGB);
	}

        unsigned char threshold = mThreshold;

	if (!isInMap(mStart))
        cout << ("AStar: Start is out of map.") << endl;
	else if(!isInMap(mGoal))
        cout << ("AStar: Goal is out of map.") << endl;
	else {
        if (getMapValue(mStart) < mThreshold)
			mThreshold = getMapValue(mStart);
        /*if (getMapValue(mGoal) < mThreshold)
            mThreshold = getMapValue(mGoal);*/
		search();
	}

        //Reset threshold for the next search
        mThreshold = threshold;

	return &this->mPath;
}

void AStar::search() {
	mNodesList->prepareNewSearch();
	mNodesList->addNode(mGoal, 0, getDistanceToStart(mGoal), NULL);
	Node* parent = mNodesList->getFirstNode();
	//If the start or the goal is in a occupied field, we have to lower
	//the occupancy threshold to find a way out.
	unsigned char threshold = mThreshold;

	while(parent->getPosition().x != mStart.x || parent->getPosition().y != mStart.y) {
		for (int i = 0; i < mNeighborCount; i++) {
			waypoint_t pose = addPosition2d(parent->getPosition(), mNeighbors[i].position);
			if (!isFreeWay(pose))
				continue;
			if (mUseRating)
				mNodesList->addNode(pose,
						parent->getDistance() + (mNeighbors[i].distance * getWayRating(pose)),
						getDistanceToStart(pose),
						parent);
			else
				mNodesList->addNode(pose,
						parent->getDistance() + (mNeighbors[i].distance),
						getDistanceToStart(pose),
						parent);
 		}
		if (mDebugImage != NULL)
			cvSet2D(this->mDebugImage, parent->getPosition().y,
					parent->getPosition().x,
					cvScalar(0,255,0));
		parent = mNodesList->getFirstNode();
		if (parent == NULL)
			break;
	}
	//Go from start backwards to the end of the path...
	Node* node = parent;
	while (node != NULL) {
		if (mDebugImage != NULL)
			cvSet2D(this->mDebugImage, node->getPosition().y,
					node->getPosition().x,
					cvScalar(0,0,255));
		this->mPath.push_back(node->getPosition());
		node = node->getParent();
	}
}

path_t* AStar::getLastPath() {
	return &this->mPath;
}

bool AStar::isInMap(const waypoint_t waypoint) {
	if (waypoint.x < 0 || waypoint.y < 0 || waypoint.x >= mMap->width || waypoint.y >= mMap->height)
		return false;
	return true;
}

bool AStar::isFreeWay(const waypoint_t waypoint) {
	if (!isInMap(waypoint))
		return false;
	uchar value = mMapPointer[(int)waypoint.y * mMap->widthStep +
	                          (int)waypoint.x];
	if (value >= mThreshold)
		return true;
	return false;
}

unsigned char AStar::getMapValue(const waypoint_t& waypoint) {
	unsigned char value = mMapPointer[(int)waypoint.y * mMap->widthStep +
	                                  (int)waypoint.x];
	return value;
}

double AStar::getWayRating(const waypoint_t waypoint) {
	uchar value = mMapPointer[(int)waypoint.y * mMap->widthStep +
	                          (int)waypoint.x];
	if (value < mThreshold || !isInMap(waypoint))
		return mMaxRating;
	uchar range = 255 - mThreshold;
	double rating = (1 - (value - mThreshold) / (double)range) * mMaxRating + 1;//((value - mThreshold) / (double)range + 1) * mMaxRating;
	return rating;
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

IplImage* AStar::getMap() {
	return this->mMap;
}

IplImage* AStar::getDebugImage() {
	return mDebugImage;
}
