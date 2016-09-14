#ifndef LOCAL_PLANNER_STAR_STATIC_H
#define LOCAL_PLANNER_STAR_STATIC_H

/// PROJECT
#include <path_follower/local_planner/local_planner_star.h>
#include <path_follower/local_planner/local_planner_static.h>

class LocalPlannerStarStatic : virtual public LocalPlannerStar, virtual public LocalPlannerStatic
{
public:
    LocalPlannerStarStatic(PathFollower& controller, tf::Transformer &transformer,
                       const ros::Duration& update_interval);
private:
    virtual void evaluate(double& current_p, double& heuristic, double& score) override;
};

#endif // LOCAL_PLANNER_STAR_STATIC_H
