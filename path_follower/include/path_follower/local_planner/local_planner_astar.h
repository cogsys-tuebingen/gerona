#ifndef LOCAL_PLANNER_ASTAR_H
#define LOCAL_PLANNER_ASTAR_H

/// PROJECT
#include <path_follower/local_planner/local_planner_star.h>

class LocalPlannerAStar : virtual public LocalPlannerStar
{
public:
    LocalPlannerAStar();
private:
    virtual double G(LNode*& current, LNode*& succ,
                     double& score) override;

    virtual void updateSucc(LNode*& current, LNode*& f_current, LNode& succ) override;
};

#endif // LOCAL_PLANNER_ASTAR_H
