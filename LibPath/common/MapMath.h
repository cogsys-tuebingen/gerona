/*
 *  Original: PlayerMath.h
 *
 *  Created on: Sep 15, 2010
 *      Author: Sebastian Scherer <sebastian.scherer@uni-tuebingen.de>
 */

#ifndef MAPMATH_H
#define MAPMATH_H

#include "Pose2d.h"

#include <cmath>
#include "MathHelper.h"

class MapInfo;

#define RTOD(r)   ((r) * 180 / M_PI)
#define DTOR(d)   ((d) * M_PI / 180)
#define NORMALIZE(z)   MathHelper::AngleClamp((z));

double NORMALIZE_0_2PI(double angle);
double NORMALIZE_0_360(double angle);

void pos2map(double px, double py, int& mx, int&my, const MapInfo& mapinfo);
void pos2map(double px, double py, double& mx, double&my, const MapInfo& mapinfo);
void map2pos(double mx, double my, double& px, double&py, const MapInfo& mapinfo);

Pose2d pos2map(const Pose2d& p, const MapInfo& mapinfo);
Pose2d map2pos(const Pose2d& p, const MapInfo& mapinfo);


#endif /* MAPMATH_H */
