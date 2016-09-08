#ifndef LOCAL_PLANNER_ASTAR_N_H
#define LOCAL_PLANNER_ASTAR_N_H

/// PROJECT
#include <path_follower/local_planner/local_planner_astar.h>
#include <path_follower/local_planner/local_planner_star_n.h>

class LocalPlannerAStarN : public LocalPlannerAStar, public LocalPlannerStarN
{
public:
    LocalPlannerAStarN(PathFollower& controller, tf::Transformer &transformer,
                       const ros::Duration& update_interval);
};

#endif // LOCAL_PLANNER_ASTAR_N_H
