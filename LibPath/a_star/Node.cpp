/*
 * Node_.cpp
 *
 *  Created on: 03.09.2010
 *      Author: dube
 */

#include "Node.h"
#include <iostream>
#include <sstream>

using namespace std;
using namespace lib_path;

Node::Node(waypoint_t position, double distance, double prediction, Node* parent, int positionIndex) {
	mPosition = position;
	mDistance = distance;
	mPrediction = prediction;
	mParent = parent;
	mVisited = false;
	mInitialized = true;
	mPositionIndex = positionIndex;
}

void Node::update(Node* node) {
	if (node->mDistance >= mDistance)
		return;
	if (mVisited)
		std::cout << "Node was already visited and should not be updated." << std::endl;
	mDistance = node->mDistance;
	mParent = node->mParent;
}

void Node::update(double distance, Node* newParent) {
	if (mVisited)
		std::cout << "Node was already visited and should not be updated." << std::endl;
	mDistance = distance;
	mParent = newParent;
}

void Node::setVisited() {
	//cout << "Set visited Node " << mPosition.x << ", " << mPosition.y << endl;
	mVisited = true;
}

int Node::getValue() const {
	//double result = mDistance + mPrediction;

	//HACK: Comparing of doubles fails in release build for unknown reasons.
	//	Anyway it works with ints and is much faster, but it might return suboptimal
	//	paths in rare cases.
	int result = (mDistance + mPrediction);
	//result = (int)(result);
	return result;
}

string Node::toString() {
	stringstream result;
	result << "Node (" << mPosition.x << ", " << mPosition.y << ")";
	result << " value = " << getValue();
	result << " visited = " << mVisited;
	return result.str();
}

void Node::reset() {
	mVisited = false;
	mInitialized = false;
}

Node::~Node() {
}
