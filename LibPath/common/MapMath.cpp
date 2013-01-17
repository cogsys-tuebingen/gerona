/*
 *  Original: PlayerMath.cpp
 *
 *  Created on: Sep 15, 2010
 *      Author: Sebastian Scherer <sebastian.scherer@uni-tuebingen.de>
 */

#include "MapMath.h"

#include "Pose2d.h"
#include "GridMap2d.h"

namespace lib_path {

void pos2map(double px, double py, double& mx, double&my, const GridMap2d& map)
{
  mx = (px - map.getOrigin().x) / map.getResolution();
  my = (py - map.getOrigin().y) / map.getResolution();
}

void map2pos(double mx, double my, double& px, double&py, const GridMap2d& map)
{
  px = map.getResolution()*mx + map.getOrigin().x;
  py = map.getResolution()*my + map.getOrigin().y;
}

Pose2d pos2map(const Pose2d& p, const GridMap2d& mapinfo)
{
  Pose2d res;
  res.theta = p.theta;
  pos2map(p.x,p.y,res.x,res.y,mapinfo);
  return res;
}

Pose2d map2pos(const Pose2d& p, const GridMap2d& mapinfo)
{
  Pose2d res;
  res.theta = p.theta;
  map2pos(p.x,p.y,res.x,res.y,mapinfo);
  return res;
}

}
