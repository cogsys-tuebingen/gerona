/*
 * Map.h
 *
 *  Created on: Apr 4, 2011
 *      Author: buck <sebastian.buck@student.uni-tuebingen.de>
 *              marks
 */

#ifndef MAPINFO_H
#define MAPINFO_H

#include <stdint.h>
#include <vector>

#include "Point2d.h"

namespace lib_path {

class MapInfo
{
public:
  MapInfo();
  bool isPosValid (const Pose2d& pos);
  bool isPosValid (const Point2d& pos);
  inline int8_t getValue(unsigned x, unsigned y) {
    return data[width * y + x];
  }
  inline void setValue(unsigned x, unsigned y, int8_t val) {
    data[width * y + x]=val;
  }

  // height of one column in data
  int height;

  // width of one row in data
  int width;

  // map resolution in meters
  float resolution;

  // all cells with values v: threshold_min <= v <= threshold_max are assumed free
  int threshold_min;
  int threshold_max;

  // coordinates of lower left corner
  Point2d origin;

  // cells
  std::vector<int8_t> data;
};

} // namespace "lib_path"

#endif // MAPINFO_H
