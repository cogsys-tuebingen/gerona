#ifndef LOCAL_PLANNER_ASTAR_N_STATIC_H
#define LOCAL_PLANNER_ASTAR_N_STATIC_H

/// PROJECT
#include <path_follower/local_planner/local_planner_astar.h>
#include <path_follower/local_planner/local_planner_star_n.h>
#include <path_follower/local_planner/local_planner_star_static.h>

class LocalPlannerAStarNStatic : public LocalPlannerAStar, public LocalPlannerStarN, public LocalPlannerStarStatic
{
public:
    LocalPlannerAStarNStatic(PathFollower& controller, tf::Transformer &transformer,
                       const ros::Duration& update_interval);
};

#endif // LOCAL_PLANNER_ASTAR_N_STATIC_H
