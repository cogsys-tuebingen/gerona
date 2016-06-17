#ifndef LOCAL_PLANNER_BFS_H
#define LOCAL_PLANNER_BFS_H

/// PROJECT
#include <path_follower/local_planner/local_planner_classic.h>

class LocalPlannerBFS : public LocalPlannerClassic
{
public:
    LocalPlannerBFS(PathFollower& controller,
                            tf::Transformer &transformer,
                            const ros::Duration& update_interval);
private:
    virtual int algo(Eigen::Vector3d& pose, SubPath& waypoints, SubPath& local_wps,
                     const std::vector<Constraint::Ptr>& constraints,
                     const std::vector<Scorer::Ptr>& scorer,
                     const std::vector<bool>& fconstraints,
                     const std::vector<double>& wscorer,
                     int& nnodes) override;
};

#endif // LOCAL_PLANNER_BFS_H
