/*
 * OpenList.cpp
 *
 *  Created on: 03.09.2010
 *      Author: dube
 */

#include <iostream>
#include <stdlib.h>
#include <iomanip>
#include <a_star/OpenList.h>


using namespace std;

OpenList::OpenList(int mapWidth, int mapHeight){
	mMapWidth = mapWidth;
	mMapHeight = mapHeight;
	mNodesArray == NULL;
	initialize();
}

OpenList::~OpenList(){
	//Delete all node
	if (mNodesArray != NULL)
		free(mNodesArray);
}

void OpenList::initialize() {
	mNodesCount = mMapWidth * mMapHeight;
	mNodesArray = (Node*)malloc(sizeof(Node) * mNodesCount);
	if (mNodesArray == NULL) {
		//Should raise an exception here.
		cout << "OpenList.cpp: Fatal Error: Could not malloc " << sizeof(Node) * mNodesCount << " bytes of memory." << endl;
		return;
	}
	Node* node = mNodesArray;
	for (int i=0; i < mNodesCount; i++) {
		node->reset();
		node++;
	}
}

void OpenList::prepareNewSearch() {
	while (!mUsedNodes.empty()) {
		(mUsedNodes.back())->reset();
		mUsedNodes.pop_back();
	}
	this->mNodeValues.clear();
	/*int nodesCount = mMapWidth * mMapHeight;
	Node* node = mNodesArray;
	for (int i=0; i < nodesCount; i++) {
		if (node->isInitialized() || node->isVisited())
			cout << "Node not reset." << endl;
		node++;
	}*/
}

void OpenList::addNode(waypoint_t position, double distance, double prediction, Node* parent) {
	//cout << "Add node: " << node->toString() << endl;
	int positionIndex = position.x + position.y * mMapWidth;
	if (positionIndex >= mNodesCount || positionIndex < 0) {
		cout << "OpenList.cpp: positionIndex is out of range. This should never happen." << endl;
		cout << "Position x = " << position.x <<
				" Position y = " << position.y <<
				" Map width = " << mMapWidth <<
				" Map height = " << mMapHeight << endl;
		return;
	}
	Node* node = &mNodesArray[positionIndex];
	if (!node->isInitialized()) {
		//This node was not visited
		//cout << "  Node is not in list. " << endl;
		mNodesArray[positionIndex] = Node(position, distance, prediction, parent, positionIndex);
		mUsedNodes.push_back(&mNodesArray[positionIndex]);
	} else {
		if (node->isVisited() || (distance + prediction) > node->getValue()) {
			//cout << "  Distance of new node is higher than the old one or the node was already visited." << endl;
			return;
		}
		//cout << "  Node is already in list, removing it." << endl;
		//Get the iterator to the node
		multiset<Node*>::iterator position = getNode(node);
		if (position == mNodeValues.end())
			cout << "OpenList.cpp: Could not find node but it should be in the ValueList." << endl;
		else
		//Remove it from the value list
			mNodeValues.erase(position);
		//Update the node with the new distance values
		node->update(distance, parent);
		//cout << "  Updated Node to " << (*element)->toString() << endl;
	}
	//Get the first element which is not smaller than the value of node
	//cout << "  Insert node to value list." << endl;
	multiset<Node*>::iterator p = mNodeValues.lower_bound(node);
	mNodeValues.insert(p, node);

	/*cout << "  Value list: ";
	for (multiset<Node*>::iterator i = mNodeValues.begin(); i != mNodeValues.end(); i++)
		cout << (*i)->getValue() << " ";
	cout << endl;

	cout << "  Position index list: ";
	for (set<Node*>::iterator i = mNodePositions.begin(); i != mNodePositions.end(); i++)
		cout << "(" << (*i)->getPosition().x << ", " << (*i)->getPosition().y << ", " << (*i)->getPositionIndex() <<  ") ";
	cout << endl;*/
}

/*set<Node*>::iterator OpenList::getNodeAtPosition(Node* node) {
	set<Node*>::iterator n = mNodePositions.lower_bound(node);
	if (n == mNodePositions.end() || (*n)->getPositionIndex != node->getPositionIndex())
		return mNodePositions.end();
	else
		n;
}*/

multiset<Node*>::iterator OpenList::getNode(Node* node) {
	multiset<Node*>::iterator lowerBound = mNodeValues.lower_bound(node);
	//cout << "Search for " << node->toString() << " "<< node->getValue() << endl;
	//cout << "  lower " << (*bounds.first)->toString() << endl;
	//cout << "  upper " << (*bounds.second)->toString() << endl;
	while ((*lowerBound) != node && lowerBound != mNodeValues.end()) {
		//cout << "  comparing to " << (*lowerBound)->toString() << endl;
		lowerBound++;
	}
	if (lowerBound == mNodeValues.end()) {
		//cout << "Search for " << node->toString() << " "<< node->getValue() << endl;
		//cout << "  lower " << (*mNodeValues.lower_bound(node))->toString() << endl;
		cout << "OpenList: getNode tried to find a node which is not in list. " <<
			"This should not happen and takes much computation time!" << endl;
		//cout << "  Value list: ";
		//	for (multiset<Node*>::iterator i = mNodeValues.begin(); i != mNodeValues.end(); i++)
		//		cout << (*i)->getValue() << " ";
		//	cout << endl;

	}
	return lowerBound;
}

Node* OpenList::getFirstNode() {
	if (!mNodeValues.empty()) {
		Node* n = *mNodeValues.begin();
		mNodeValues.erase(mNodeValues.begin());
		n->setVisited();
		return n;
	}
	else
		return NULL;
}

void OpenList::checkList() {
	cout << "Check list" << endl;
	multiset<Node*>::iterator i = mNodeValues.begin();
	Node* first = (*i);
	i++;
	while (i != mNodeValues.end()) {
		if (first->getValue() > (*i)->getValue()) {
			cout << "  OpenList: Error: List not sorted!!!" << endl;
			cout << first->getValue() << " <=! " << (*i)->getValue() << endl;
			printList();
		}
		//cout << "  " << first->getValue() << "< " << (*i)->getValue() << endl;
		first = (*i);
		i++;
	}
}

void OpenList::printList() {
	cout << "Print list" << endl;
	multiset<Node*>::iterator i = mNodeValues.begin();
	while (i != mNodeValues.end()) {
		cout << (*i)->getValue() << " ";
		i++;
	}
	cout << endl;
}
