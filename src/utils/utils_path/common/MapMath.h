/*
 *  Original: PlayerMath.h
 *
 *  Created on: Sep 15, 2010
 *      Author: Sebastian Scherer <sebastian.scherer@uni-tuebingen.de>
 */

#ifndef MAPMATH_H
#define MAPMATH_H

#include <cmath>
#include <utils_general/MathHelper.h>

#define RTOD(r)   ((r) * 180 / M_PI)
#define DTOR(d)   ((d) * M_PI / 180)
#define NORMALIZE(z) MathHelper::AngleClamp((z))

namespace lib_path {
class GridMap2d;
class Pose2d;

double NORMALIZE_0_2PI(double angle);
double NORMALIZE_0_360(double angle);

void pos2map(double px, double py, int& mx, int&my, const GridMap2d& mapinfo);
void pos2map(double px, double py, double& mx, double&my, const GridMap2d& mapinfo);
void map2pos(double mx, double my, double& px, double&py, const GridMap2d& mapinfo);

Pose2d pos2map(const Pose2d& p, const GridMap2d& mapinfo);
Pose2d map2pos(const Pose2d& p, const GridMap2d& mapinfo);

} // namespace "lib_path"

#endif /* MAPMATH_H */
