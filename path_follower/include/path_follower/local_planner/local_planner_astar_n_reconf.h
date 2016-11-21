#ifndef LOCAL_PLANNER_ASTAR_N_RECONF_H
#define LOCAL_PLANNER_ASTAR_N_RECONF_H

/// PROJECT
#include <path_follower/local_planner/local_planner_astar.h>
#include <path_follower/local_planner/local_planner_star_n.h>
#include <path_follower/local_planner/local_planner_star_reconf.h>

class LocalPlannerAStarNReconf : public LocalPlannerAStar, public LocalPlannerStarN, public LocalPlannerStarReconf
{
public:
    LocalPlannerAStarNReconf();
};

#endif // LOCAL_PLANNER_ASTAR_N_RECONF_H
