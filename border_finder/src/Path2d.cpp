/*
 * Path2d.cpp
 *
 *  Created on: 07.09.2010
 *      Author: dube
 */

#include <iostream>
#include <cmath>
#include "Path2d.h"

using namespace std;

Path2d::Path2d():
    vector<pose2d_t>() {
}

Path2d::~Path2d() {

}

void Path2d::flattenPath(double flattenThreshold) {
	//cout << "Flatten..." << endl;
	if (size() < 3)
		return;
	mFlattenThreshold = flattenThreshold;
	mFlattenPath.clear();
	//Add first and last Node to list.
	mFlattenPath.push_back(0);
	mFlattenPath.push_back(size() - 1);

	list<int>::iterator i = mFlattenPath.begin();
	i++;
	while (i != mFlattenPath.end()) {
		bool again = checkRange(i);
		if (again)
			i--;
		else
			i++;
	}

	Path2d newPath;
	for (list<int>::iterator i = mFlattenPath.begin(); i != mFlattenPath.end(); i++)
		newPath.push_back((*this)[*i]);
	clear();
	for (Path2d::iterator i = newPath.begin(); i != newPath.end(); i++)
		push_back(*i);
	/*for (Path2d::iterator i = newPath.end(); i != newPath.begin(); ) {
		i--;
		push_back(*i);
	}*/
}

bool Path2d::checkRange(list<int>::iterator i) {
	int secondElement = (*i);
	int firstElement = (*(--i));
	i++;
	if (secondElement - firstElement < 3)
		return false;
	//cout << "Check Element " << firstElement << ":" << secondElement << endl;
	//cout << "Maxdistance " << mFlattenThreshold << endl;
	//Precalculate some values for the distance calculation
	//Calculate the normalized vector between the two points
  pose2d_t p1 = (*this)[firstElement];
  pose2d_t p2 = (*this)[secondElement];
	double vx = p2.px - p1.px;
	double vy = p2.py - p1.py;
	double l = sqrt(pow(vx,2) + pow(vy,2));
	/*if (l < 0.2)
		cout << "Path2d: There seems to be a bug." << vx << ", " << vy << endl;*/
	vx = vx / l;
	vy = vy / l;
	//Calculate divisor
	double div = pow(vx,2) + pow(vy,2);
	//Iterate over all elements and find the point with the biggest distance.
	double maxDist = 0;
	int maxElement = -1;
	for (int element = firstElement + 1; element < secondElement; element++) {
    pose2d_t q = (*this)[element];
		double distance = fabs((vx *(p2.py - q.py) + vy * q.px - vy * p2.px) / div);
		//cout << "Distance = " << distance << endl;
		if (distance > maxDist) {
			maxDist = distance;
			maxElement = element;
		}
	}
	//Is the maximum distance higher than the Threshold?
	if (maxElement == -1 || maxDist < mFlattenThreshold)
		return false;
	mFlattenPath.insert(i, maxElement);
	return true;
}
