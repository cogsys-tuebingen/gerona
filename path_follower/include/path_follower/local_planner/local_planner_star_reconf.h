#ifndef LOCAL_PLANNER_STAR_RECONF_H
#define LOCAL_PLANNER_STAR_RECONF_H

/// PROJECT
#include <path_follower/local_planner/local_planner_star.h>
#include <path_follower/local_planner/local_planner_reconf.h>

class LocalPlannerStarReconf : virtual public LocalPlannerStar, virtual public LocalPlannerReconf
{
public:
    LocalPlannerStarReconf(PathFollower& controller, tf::Transformer &transformer,
                       const ros::Duration& update_interval);
private:
    virtual void evaluate(double& current_p, double& heuristic, double& score) override;
};

#endif // LOCAL_PLANNER_STAR_RECONF_H
