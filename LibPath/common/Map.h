/*
 * Map.h
 *
 *  Created on: Apr 4, 2011
 *      Author: buck <sebastian.buck@student.uni-tuebingen.de>
 */

#ifndef MAPINFO_H
#define MAPINFO_H

#include "Point2d.h"

#include <stdint.h>
#include <vector>

class MapInfo
{
public:
  MapInfo();

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

#endif // MAPINFO_H
