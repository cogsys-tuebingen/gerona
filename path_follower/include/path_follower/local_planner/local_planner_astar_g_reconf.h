#ifndef LOCAL_PLANNER_ASTAR_G_RECONF_H
#define LOCAL_PLANNER_ASTAR_G_RECONF_H

/// PROJECT
#include <path_follower/local_planner/local_planner_astar.h>
#include <path_follower/local_planner/local_planner_star_g.h>
#include <path_follower/local_planner/local_planner_star_reconf.h>

class LocalPlannerAStarGReconf : public LocalPlannerAStar, public LocalPlannerStarG, public LocalPlannerStarReconf
{
public:
    LocalPlannerAStarGReconf(RobotController& controller, PoseTracker& pose_tracker,
                             const ros::Duration& update_interval);
};

#endif // LOCAL_PLANNER_ASTAR_G_RECONF_H
