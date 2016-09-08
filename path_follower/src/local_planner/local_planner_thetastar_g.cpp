/// HEADER
#include <path_follower/local_planner/local_planner_thetastar_g.h>

/// PROJECT
#include <path_follower/pathfollower.h>

LocalPlannerThetaStarG::LocalPlannerThetaStarG(PathFollower &follower,
                                               tf::Transformer& transformer,
                                               const ros::Duration& update_interval)
    : LocalPlannerStar(follower, transformer, update_interval),
      LocalPlannerThetaStar(follower, transformer, update_interval),
      LocalPlannerStarG(follower, transformer, update_interval)
{

}
