#ifndef LOCAL_PLANNER_THETASTAR_N_RECONF_H
#define LOCAL_PLANNER_THETASTAR_N_RECONF_H

/// PROJECT
#include <path_follower/local_planner/local_planner_thetastar.h>
#include <path_follower/local_planner/local_planner_star_n.h>
#include <path_follower/local_planner/local_planner_star_reconf.h>

class LocalPlannerThetaStarNReconf : public LocalPlannerThetaStar, public LocalPlannerStarN, public LocalPlannerStarReconf
{
public:
    LocalPlannerThetaStarNReconf(RobotController& controller, PoseTracker& pose_tracker,
                                 const ros::Duration& update_interval);
};

#endif // LOCAL_PLANNER_THETASTAR_N_RECONF_H
