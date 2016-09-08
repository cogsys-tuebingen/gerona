#ifndef LOCAL_PLANNER_THETASTAR_N_H
#define LOCAL_PLANNER_THETASTAR_N_H

/// PROJECT
#include <path_follower/local_planner/local_planner_thetastar.h>
#include <path_follower/local_planner/local_planner_star_n.h>

class LocalPlannerThetaStarN : public LocalPlannerThetaStar, public LocalPlannerStarN
{
public:
    LocalPlannerThetaStarN(PathFollower& controller, tf::Transformer &transformer,
                       const ros::Duration& update_interval);
};

#endif // LOCAL_PLANNER_THETASTAR_N_H
