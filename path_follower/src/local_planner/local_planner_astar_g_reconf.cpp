/// HEADER
#include <path_follower/local_planner/local_planner_astar_g_reconf.h>

/// PROJECT
#include <path_follower/pathfollower.h>

LocalPlannerAStarGReconf::LocalPlannerAStarGReconf(PathFollower &follower,
                                       tf::Transformer& transformer,
                                       const ros::Duration& update_interval)
    : LocalPlannerClassic(follower, transformer, update_interval),
      LocalPlannerStar(follower, transformer, update_interval),
      LocalPlannerReconf(follower, transformer, update_interval),
      LocalPlannerAStar(follower, transformer, update_interval),
      LocalPlannerStarG(follower, transformer, update_interval),
      LocalPlannerStarReconf(follower, transformer, update_interval)
{

}
