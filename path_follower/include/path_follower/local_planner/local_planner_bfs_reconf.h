#ifndef LOCAL_PLANNER_BFS_RECONF_H
#define LOCAL_PLANNER_BFS_RECONF_H

/// PROJECT
#include <path_follower/local_planner/local_planner_bfs.h>
#include <path_follower/local_planner/local_planner_reconf.h>

class LocalPlannerBFSReconf : public LocalPlannerBFS, public LocalPlannerReconf
{
public:
    LocalPlannerBFSReconf(PathFollower& controller, tf::Transformer &transformer,
                       const ros::Duration& update_interval);
};

#endif // LOCAL_PLANNER_BFS_RECONF_H
