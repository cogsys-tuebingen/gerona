/// HEADER
#include <path_follower/local_planner/local_planner_thetastar_n.h>

/// PROJECT
#include <path_follower/pathfollower.h>

LocalPlannerThetaStarN::LocalPlannerThetaStarN(PathFollower &follower,
                                               tf::Transformer& transformer,
                                               const ros::Duration& update_interval)
    : LocalPlannerStar(follower, transformer, update_interval),
      LocalPlannerThetaStar(follower, transformer, update_interval),
      LocalPlannerStarN(follower, transformer, update_interval)
{

}
