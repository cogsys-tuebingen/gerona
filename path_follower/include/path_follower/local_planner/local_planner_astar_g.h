#ifndef LOCAL_PLANNER_ASTAR_G_H
#define LOCAL_PLANNER_ASTAR_G_H

/// PROJECT
#include <path_follower/local_planner/local_planner_astar.h>

class LocalPlannerAStarG : public LocalPlannerAStar
{
public:
    LocalPlannerAStarG(PathFollower& controller,
                            tf::Transformer &transformer,
                            const ros::Duration& update_interval);
private:
    virtual double f(double& g, double& score, double& heuristic) override;
};

#endif // LOCAL_PLANNER_ASTAR_G_H
