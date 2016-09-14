/// HEADER
#include <path_follower/local_planner/local_planner_thetastar_n_static.h>

/// PROJECT
#include <path_follower/pathfollower.h>

LocalPlannerThetaStarNStatic::LocalPlannerThetaStarNStatic(PathFollower &follower,
                                               tf::Transformer& transformer,
                                               const ros::Duration& update_interval)
    : LocalPlannerClassic(follower, transformer, update_interval),
      LocalPlannerStar(follower, transformer, update_interval),
      LocalPlannerStatic(follower, transformer, update_interval),
      LocalPlannerThetaStar(follower, transformer, update_interval),
      LocalPlannerStarN(follower, transformer, update_interval),
      LocalPlannerStarStatic(follower, transformer, update_interval)
{

}
