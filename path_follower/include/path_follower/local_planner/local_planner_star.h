#ifndef LOCAL_PLANNER_STAR_H
#define LOCAL_PLANNER_STAR_H

/// PROJECT
#include <path_follower/local_planner/local_planner_classic.h>

class LocalPlannerStar : public LocalPlannerClassic
{
public:
    LocalPlannerStar(PathFollower& controller,
                            tf::Transformer &transformer,
                            const ros::Duration& update_interval);
private:
    virtual bool algo(Eigen::Vector3d& pose, SubPath& local_wps,
                     const std::vector<Constraint::Ptr>& constraints,
                     const std::vector<Scorer::Ptr>& scorer,
                     const std::vector<bool>& fconstraints,
                     const std::vector<double>& wscorer,
                     std::size_t& nnodes) override;
    virtual double f(double& g, double& score, double& heuristic) = 0;

    virtual double G(LNode*& current, std::size_t& index, std::vector<LNode*>& successors,
             const std::vector<Scorer::Ptr>& scorer, const std::vector<double>& wscorer,
             double& score) = 0;

    virtual void updateSucc(LNode*& current, LNode*& f_current, LNode& succ) = 0;

private:
    typedef std::multiset<LNode*,CompareHNode> prio_queue;

protected://TMP
    virtual void initLeaves(LNode& root) override;

    virtual void updateLeaves(std::vector<LNode*>& successors, LNode*& current) override;

    virtual void updateBest(double& current_p, double& best_p, LNode*& obj, LNode*& succ) override;

    virtual void addLeaf(LNode*& node) override;

    virtual void reconfigureTree(LNode*& obj, std::vector<LNode>& nodes,  double& best_p,
                                 const std::vector<Scorer::Ptr>& scorer,
                                 const std::vector<double>& wscorer) override;
};

#endif // LOCAL_PLANNER_STAR_H
