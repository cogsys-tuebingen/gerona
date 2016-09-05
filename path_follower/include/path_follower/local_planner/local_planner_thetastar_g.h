#ifndef LOCAL_PLANNER_THETASTAR_G_H
#define LOCAL_PLANNER_THETASTAR_G_H

/// PROJECT
#include <path_follower/local_planner/local_planner_thetastar.h>
#include <path_follower/local_planner/local_planner_star_g.h>

class LocalPlannerThetaStarG : public LocalPlannerThetaStar, public LocalPlannerStarG
{
public:
    LocalPlannerThetaStarG(PathFollower& controller, tf::Transformer &transformer,
                       const ros::Duration& update_interval);
};

#endif // LOCAL_PLANNER_THETASTAR_G_H
