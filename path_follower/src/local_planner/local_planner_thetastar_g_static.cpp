/// HEADER
#include <path_follower/local_planner/local_planner_thetastar_g_static.h>

/// PROJECT
#include <path_follower/pathfollower.h>

LocalPlannerThetaStarGStatic::LocalPlannerThetaStarGStatic(PathFollower &follower,
                                               tf::Transformer& transformer,
                                               const ros::Duration& update_interval)
    : LocalPlannerClassic(follower, transformer, update_interval),
      LocalPlannerStar(follower, transformer, update_interval),
      LocalPlannerStatic(follower, transformer, update_interval),
      LocalPlannerThetaStar(follower, transformer, update_interval),
      LocalPlannerStarG(follower, transformer, update_interval),
      LocalPlannerStarStatic(follower, transformer, update_interval)
{

}
