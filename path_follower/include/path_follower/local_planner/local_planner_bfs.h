#ifndef LOCAL_PLANNER_BFS_H
#define LOCAL_PLANNER_BFS_H

/// PROJECT
#include <path_follower/local_planner/local_planner_classic.h>

class LocalPlannerBFS : virtual public LocalPlannerClassic
{
public:
    LocalPlannerBFS(PathFollower& controller,
                            tf::Transformer &transformer,
                            const ros::Duration& update_interval);
private:
    virtual bool algo(Eigen::Vector3d& pose, SubPath& local_wps,
                     const std::vector<Constraint::Ptr>& constraints,
                     const std::vector<Scorer::Ptr>& scorer,
                     const std::vector<bool>& fconstraints,
                     const std::vector<double>& wscorer,
                     std::size_t& nnodes) override;
    virtual void evaluate(double& current_p, LNode*& succ, double& dis2last,
                          const std::vector<Scorer::Ptr>& scorer,
                          const std::vector<double>& wscorer) = 0;
};

#endif // LOCAL_PLANNER_BFS_H
