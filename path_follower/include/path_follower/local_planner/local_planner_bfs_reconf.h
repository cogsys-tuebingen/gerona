#ifndef LOCAL_PLANNER_BFS_RECONF_H
#define LOCAL_PLANNER_BFS_RECONF_H

/// PROJECT
#include <path_follower/local_planner/local_planner_bfs.h>
#include <path_follower/local_planner/local_planner_reconf.h>

class LocalPlannerBFSReconf : public LocalPlannerBFS, public LocalPlannerReconf
{
public:
    LocalPlannerBFSReconf(RobotController& controller, PoseTracker& pose_tracker,
                          const ros::Duration& update_interval);
private:
    virtual void evaluate(double& current_p, LNode*& succ, double& dis2last,
                          const std::vector<Scorer::Ptr>& scorer,
                          const std::vector<double>& wscorer) override;
};

#endif // LOCAL_PLANNER_BFS_RECONF_H
