/*
 * Pose2d.cpp
 *
 *  Created on: Apr 4, 2011
 *      Author: buck <sebastian.buck@student.uni-tuebingen.de>
 */


#include "../../LibUtil/MathHelper.h"

#include "Pose2d.h"

using namespace lib_path;
using namespace std;

Pose2d::Pose2d()
  : x(0), y(0), theta(0)
{
}

Pose2d::Pose2d(double x, double y, double theta)
  : x(x), y(y), theta(theta)
{
}

double Pose2d::distance_to(const Pose2d &rhs) const
{
  return sqrt(pow(x-rhs.x, 2) + pow(y-rhs.y, 2));
}

double Pose2d::distance_to_origin() const
{
  return sqrt(pow(x, 2) + pow(y, 2));
}

bool Pose2d::isEqual( const Pose2d &p,
                      const double dist_eps,
                      const double theta_eps ) const
{
    return distance_to( p ) <= dist_eps
            && fabs( MathHelper::AngleDelta( theta, p.theta )) <= theta_eps;
}
