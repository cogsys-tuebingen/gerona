#ifndef LOCAL_PLANNER_THETASTAR_G_STATIC_H
#define LOCAL_PLANNER_THETASTAR_G_STATIC_H

/// PROJECT
#include <path_follower/local_planner/local_planner_thetastar.h>
#include <path_follower/local_planner/local_planner_star_g.h>
#include <path_follower/local_planner/local_planner_star_static.h>

class LocalPlannerThetaStarGStatic : public LocalPlannerThetaStar, public LocalPlannerStarG, public LocalPlannerStarStatic
{
public:
    LocalPlannerThetaStarGStatic();
};

#endif // LOCAL_PLANNER_THETASTAR_G_STATIC_H
