/*
 *  Original: PlayerMath.cpp
 *
 *  Created on: Sep 15, 2010
 *      Author: Sebastian Scherer <sebastian.scherer@uni-tuebingen.de>
 */

#include "MapMath.h"

#include "Map.h"

namespace lib_path {

double NORMALIZE_0_2PI(double angle)
{
  while (angle > 2 * M_PI)
    angle -= 2 * M_PI;
  while (angle < 0)
    angle += 2 * M_PI;

  return angle;
}

double NORMALIZE_0_360(double angle)
{
  while (angle > 360)
    angle -= 360;
  while (angle < 0)
    angle += 360;

  return angle;
}

void pos2map(double px, double py, double& mx, double&my, const MapInfo& mapinfo)
{
  mx = (px - mapinfo.origin.x) / mapinfo.resolution;
  my = (py - mapinfo.origin.y) / mapinfo.resolution;
}

void map2pos(double mx, double my, double& px, double&py, const MapInfo& mapinfo)
{
  px = mapinfo.resolution*mx + mapinfo.origin.x;
  py = mapinfo.resolution*my + mapinfo.origin.y;
}

Pose2d pos2map(const Pose2d& p, const MapInfo& mapinfo)
{
  Pose2d res;
  res.theta = p.theta;
  pos2map(p.x,p.y,res.x,res.y,mapinfo);
  return res;
}

Pose2d map2pos(const Pose2d& p, const MapInfo& mapinfo)
{
  Pose2d res;
  res.theta = p.theta;
  map2pos(p.x,p.y,res.x,res.y,mapinfo);
  return res;
}

}
