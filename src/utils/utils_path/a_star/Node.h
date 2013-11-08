/*
 * Node_.h
 *
 *  Created on: 03.09.2010
 *      Author: dube
 */

#ifndef NODE_H_
#define NODE_H_

#include <string>

namespace lib_path {

struct waypoint_t {
	int x;
	int y;
};

class Node {
public:
	Node(waypoint_t position, double distance, double prediction, Node* parent, int positionIndex);
	virtual ~Node();
	void update(Node* node);
	void update(double distance, Node* newParent);
	Node *getParent() const {return mParent;}
    void setParent(Node *mParent) { this->mParent = mParent;}
    waypoint_t getPosition() const { return mPosition;}
    int getValue() const;
    double getDistance() const { return mDistance; }
    void setVisited();
    void reset();
    bool isVisited() const { return mVisited; }
    bool isInitialized() const { return mInitialized; }
    int getPositionIndex() const {return mPositionIndex;}
    std::string toString();

private:
	waypoint_t mPosition;
    float mDistance;
    float mPrediction;
	Node* mParent;
	bool mVisited;
	bool mInitialized;
	int mPositionIndex;
};

} // Namespace "lib_path"

#endif /* NODE_H_ */
