/*
 * Just a small program to test some functions of the path planner.
 */

#include <iostream>
#include "Segment.h"
#include <libplayercore/playercore.h>
#include <cmath>
#include <string>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <Stopwatch.h>
#include <set>
#include "AStar.h"
#include "Node.h"

using namespace std;

bool testEqualEpsilon(double value1, double value2) {
	double epsilon = 0.000000001;
	if (value1 > value2 - epsilon && value1 < value2 + epsilon)
		return true;
	else return false;
}

int testSegment() {
	player_pose2d_t p1;
	player_pose2d_t p2;
	p1.px = 5;
	p1.py = 5;
	p2.px = 10;
	p2.py = 10;

	Segment s = Segment(p1,p2);

	player_pose2d_t p;

	p.px = 0;
	p.py = 0;
	double dist = s.getDistanceToLine(p);
	if (!testEqualEpsilon(dist, sqrt(50))) {
		cout << "Should be " << sqrt(50) << " is " << dist << endl;
		return 1;
	}

	p.px = 5;
	p.py = 10;
	dist = s.getDistanceToLine(p);
	double result = sqrt(2.5 * 2.5 * 2);
	if (!testEqualEpsilon(dist, result)) {
		cout << "Should be " << result << " is " << dist << endl;
		return 1;
	}

	return 0;
}

int testPathPlanning() {
	string mapfile = "/home/dube/Dokumente/Pictures/Maps/map1.png";
	IplImage* i = cvLoadImage(mapfile.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
	waypoint_t start;
	waypoint_t end;
	start.x = 900;
	start.y = 980;
	end.x = 800;
	end.y = 200;
	/*start.x = 380;
	start.y = 450;
	end.x = 380;
	end.y = 400;*/
	Stopwatch stop;
	AStar a = AStar(i, 1, 5, true);
	a.planPath(start, end);
	cout << "PathPlanning took " << stop.sElapsedDouble() << "." << endl;
	cvShowImage("Path", a.getMap());
	cvWaitKey(0);
	stop.restart();
	a.setNewMap(i);
	a.planPath(start, end);
	cout << "PathPlanning took " << stop.sElapsedDouble() << "." << endl;
	cvShowImage("Path", a.getMap());
	cvWaitKey(0);
	stop.restart();
	cvReleaseImage(&i);
	/*mapfile = "/home/dube/Dokumente/Pictures/Maps/map3.png";
	i = cvLoadImage(mapfile.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
	a.setNewMap(i);
	cout << "Setting new map took " << stop.sElapsedDouble() << "." << endl;
	a.planPath(start, end);
	cout << "PathPlanning took " << stop.sElapsedDouble() << "." << endl;
	cvShowImage("Path", a.getMap());
	cvWaitKey(0);
	cvReleaseImage(&i);*/
	return 0;
}

struct NodePositionCompare {
	bool operator() (const Node* lhs, const Node* rhs) const
		{return lhs->getPositionIndex() < rhs->getPositionIndex();}
};
struct NodeValueCompare {
	bool operator() (const Node* lhs, const Node* rhs) const
		{return lhs->getValue() < rhs->getValue();}
};

void testSpeedOfSet() {
	std::multiset<Node*, NodeValueCompare> mNodeValues;
	vector<Node*> Nodes;
	string mapfile = "/home/dube/Dokumente/Pictures/Maps/map1.png";
	IplImage* image = cvLoadImage(mapfile.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
	srand ( time(NULL) );
	const int n = image->width * image->height;

	Stopwatch w;
	for (int i = 0; i < n; i++) {
		waypoint_t w;
		w.x = rand() % image->width;
		w.y = rand() % image->height;
		double distance = rand() / (double)RAND_MAX * 50;
		double prediction = rand() / (double)RAND_MAX * 50;
		Node* n = new Node(w, distance, prediction, NULL, w.x + w.y * image->widthStep);
		mNodeValues.insert(n);
		Nodes.push_back(n);
	}
	cout << "Creation of " << n << " nodes took " << w.sElapsedDouble() << " seconds." << endl;

	Node* nodes_vec = (Node*)malloc(sizeof(Node)*n);
	w.restart();
	for (int i = 0; i < n; i++) {
		nodes_vec[0].setVisited();
		Nodes.push_back(&nodes_vec[0]);
		nodes_vec++;
	}
	cout << "Creation of array with " << n << " nodes took " << w.sElapsedDouble() << " seconds." << endl;

	w.restart();
	for (int i = 0; i < n; i++) {
		int index = rand()  % (n-1);
		Node* node = Nodes[index];
		mNodeValues.find(node);
	}
	cout << "Search of " << n << " nodes took " << w.sElapsedDouble() << " seconds." << endl;
	w.restart();
	multiset<Node*>::iterator it;
	for (int i = 0; i < n; i++) {
		int index = rand()  % (n-1);
		Node* node = Nodes[index];
		it = mNodeValues.lower_bound(node);
		while((*it) != node)
			it++;
	}
	cout << "Search method 2 for " << n << " nodes took " << w.sElapsedDouble() << " seconds." << endl;

}

int testMultiset() {
	std::multiset<Node*, NodeValueCompare> mNodeValues;
	vector<Node*> Nodes;
	string mapfile = "/home/dube/Dokumente/Pictures/Maps/map1.png";
	IplImage* image = cvLoadImage(mapfile.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
	srand ( time(NULL) );
	const int n = image->width * image->height / 100;

	Stopwatch w;
	for (int i = 0; i < n; i++) {
		waypoint_t w;
		w.x = rand() % image->width;
		w.y = rand() % image->height;
		double distance = rand() / (double)RAND_MAX * 50;
		double prediction = rand() / (double)RAND_MAX * 50;
		Node* n = new Node(w, distance, prediction, NULL, w.x + w.y * image->widthStep);
		mNodeValues.insert(n);
		Nodes.push_back(n);
	}
	cout << "Creation of " << n << " nodes took " << w.sElapsedDouble() << " seconds." << endl;

	for (int i = 0; i < n; i++) {
		int index = rand() % n;
		Node* node = Nodes[index];
		multiset<Node*>::iterator lowerBound = mNodeValues.lower_bound(node);
		while ((*lowerBound) != node && lowerBound != mNodeValues.end()) {
			lowerBound ++;
		}
		if (lowerBound == mNodeValues.end()) {
			for (multiset<Node*>::iterator it = mNodeValues.begin();
					it != mNodeValues.end(); it++)
				cout << (*it)->getValue() << " ";
			cout << endl << "Searching for " << node->toString() << endl;
			cout << "Lower Bound: " << (*mNodeValues.lower_bound(node))->toString() << endl;
			cout << endl;
		}
	}
}

int
main(int argc, char *argv[])
{
	int error;

	/*error = testSegment();
	if (error > 0)
		cout << "Error in Segment class: Test " << error << " failed." << endl;
	else cout << "Segment Class OK." << endl;

	testSpeedOfSet();*/

	testMultiset();

	error = testPathPlanning();
	if (error > 0)
		cout << "Error in AStar class: Test " << error << " failed." << endl;
	else cout << "AStar Class OK." << endl;
}
