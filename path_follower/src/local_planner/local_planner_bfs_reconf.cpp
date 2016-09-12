/// HEADER
#include <path_follower/local_planner/local_planner_bfs_reconf.h>

/// PROJECT
#include <path_follower/pathfollower.h>

LocalPlannerBFSReconf::LocalPlannerBFSReconf(PathFollower &follower,
                                       tf::Transformer& transformer,
                                       const ros::Duration& update_interval)
    : LocalPlannerClassic(follower, transformer, update_interval),
      LocalPlannerBFS(follower, transformer, update_interval),
      LocalPlannerReconf(follower, transformer, update_interval)
{

}
