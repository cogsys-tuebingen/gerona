#ifndef LOCAL_PLANNER_ASTAR_G_H
#define LOCAL_PLANNER_ASTAR_G_H

/// PROJECT
#include <path_follower/local_planner/local_planner_astar.h>
#include <path_follower/local_planner/local_planner_star_g.h>

class LocalPlannerAStarG : public LocalPlannerAStar, public LocalPlannerStarG
{
public:
    LocalPlannerAStarG(PathFollower& controller, tf::Transformer &transformer,
                       const ros::Duration& update_interval);
};

#endif // LOCAL_PLANNER_ASTAR_G_H
