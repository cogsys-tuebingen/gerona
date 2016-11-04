#ifndef LOCAL_PLANNER_THETASTAR_N_STATIC_H
#define LOCAL_PLANNER_THETASTAR_N_STATIC_H

/// PROJECT
#include <path_follower/local_planner/local_planner_thetastar.h>
#include <path_follower/local_planner/local_planner_star_n.h>
#include <path_follower/local_planner/local_planner_star_static.h>

class LocalPlannerThetaStarNStatic : public LocalPlannerThetaStar, public LocalPlannerStarN, public LocalPlannerStarStatic
{
public:
    LocalPlannerThetaStarNStatic();
};

#endif // LOCAL_PLANNER_THETASTAR_N_STATIC_H
