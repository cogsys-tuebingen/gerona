/*
 * OpenList.h
 *
 *  Created on: 03.09.2010
 *      Author: dube
 */

#ifndef OPENLIST_H_
#define OPENLIST_H_

#include <list>
#include <set>
#include <vector>
#include "Node.h"

namespace lib_path {

class OpenList{
public:
	OpenList(int mapWidth, int mapHeight);
	virtual ~OpenList();
	void addNode(waypoint_t position, double distance, double prediction, Node* parent);
	bool isWaypointInList(const waypoint_t waypoint);
	std::multiset<Node*>::iterator getNode(Node* node);
	/*Node* getNodeByPosition(Node* node);*/
	Node* getFirstNode();
	void prepareNewSearch();
	void checkList();
	void printList();

private:
	struct NodeValueCompare {
		bool operator() (const Node* lhs, const Node* rhs) const
			{return (lhs->getValue() < rhs->getValue());}
	};

	void initialize();

	Node* mNodesArray;
	std::multiset<Node*, NodeValueCompare> mNodeValues;
	std::vector<Node*> mUsedNodes;
	int mMapWidth;
	int mMapHeight;
	int mWidthStep;
	int mNodesCount;
};

} // Namespace "lib_path"

#endif /* OPENLIST_H_ */
