#ifndef LOCAL_PLANNER_THETASTAR_G_RECONF_H
#define LOCAL_PLANNER_THETASTAR_G_RECONF_H

/// PROJECT
#include <path_follower/local_planner/local_planner_thetastar.h>
#include <path_follower/local_planner/local_planner_star_g.h>
#include <path_follower/local_planner/local_planner_star_reconf.h>

class LocalPlannerThetaStarGReconf : public LocalPlannerThetaStar, public LocalPlannerStarG, public LocalPlannerStarReconf
{
public:
    LocalPlannerThetaStarGReconf(PathFollower& controller, tf::Transformer &transformer,
                       const ros::Duration& update_interval);
};

#endif // LOCAL_PLANNER_THETASTAR_G_RECONF_H
