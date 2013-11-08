/*
 * Path2d.h
 *
 *  Created on: 07.09.2010
 *      Author: dube
 */

#ifndef PATH2D_H_
#define PATH2D_H_

#include <vector>
#include <list>

struct pose2d_t {
  double px,py, pa;
};

class Path2d : public std::vector<pose2d_t>{
public:
	Path2d();
	virtual ~Path2d();
	void flattenPath(double flattenThreshold);

private:
	bool checkRange(std::list<int>::iterator i);
	double mFlattenThreshold;
	std::list<int> mFlattenPath;
};

#endif /* PATH2D_H_ */
