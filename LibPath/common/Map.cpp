/*
 * Map.cpp
 *
 *  Created on: Apr 4, 2011
 *      Author: buck <sebastian.buck@student.uni-tuebingen.de>
 */

#include "Map.h"

using namespace lib_path;

MapInfo::MapInfo()
{
}

bool MapInfo::isPosValid(const Point2d &pos)
{
  return (pos.x>=0.0 && pos.x<width && pos.y>=0.0 && pos.y<height);
}


bool MapInfo::isPosValid(const Pose2d &pos)
{
  return (pos.x>=0.0 && pos.x<width && pos.y>=0.0 && pos.y<height);
}
