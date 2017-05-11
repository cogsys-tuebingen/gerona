#ifndef LOCAL_PLANNER_RECONF_H
#define LOCAL_PLANNER_RECONF_H

/// PROJECT
#include <path_follower/local_planner/high_speed/local_planner_classic.h>

class LocalPlannerReconf : virtual public LocalPlannerClassic
{
public:
    LocalPlannerReconf();

protected:
    virtual void initLeaves(LNode& root) override;

    virtual void updateLeaves(std::vector<LNode*>& successors, LNode*& current) override;

    virtual void updateBest(double& current_p, double& best_p, LNode*& obj, LNode*& succ) override;

    virtual void addLeaf(LNode*& node) override;

    virtual void reconfigureTree(LNode*& obj, std::vector<LNode>& nodes, double& best_p) override;

private:
    std::vector<LNode*> leaves;
};

#endif // LOCAL_PLANNER_RECONF_H
