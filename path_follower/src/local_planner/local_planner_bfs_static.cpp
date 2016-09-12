/// HEADER
#include <path_follower/local_planner/local_planner_bfs_static.h>

/// PROJECT
#include <path_follower/pathfollower.h>

LocalPlannerBFSStatic::LocalPlannerBFSStatic(PathFollower &follower,
                                       tf::Transformer& transformer,
                                       const ros::Duration& update_interval)
    : LocalPlannerClassic(follower, transformer, update_interval),
      LocalPlannerBFS(follower, transformer, update_interval),
      LocalPlannerStatic(follower, transformer, update_interval)
{

}
