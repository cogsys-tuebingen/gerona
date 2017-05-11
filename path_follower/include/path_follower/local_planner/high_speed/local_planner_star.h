#ifndef LOCAL_PLANNER_STAR_H
#define LOCAL_PLANNER_STAR_H

/// PROJECT
#include <path_follower/local_planner/high_speed/local_planner_classic.h>

class LocalPlannerStar : virtual public LocalPlannerClassic
{
public:
    LocalPlannerStar();
private:
    virtual void setInitScores(LNode& wpose, double& dis2last) override;

    virtual void initQueue(LNode& root) override;

    virtual bool isQueueEmpty() override;

    virtual LNode* queueFront() override;

    virtual void pop(LNode*& current) override;

    virtual void push2Closed(LNode*& current) override;

    virtual void expandCurrent(LNode*& current, std::size_t& nsize, std::vector<LNode*>& successors,
                               std::vector<LNode>& nodes) override;

    virtual bool processSuccessor(LNode*& succ, LNode*& current,
                                  double& current_p,double& dis2last) override;

    virtual double f(double& g, double& score, double& heuristic) = 0;

    virtual double G(LNode*& current, LNode*& succ,
                     double& score) = 0;

    virtual void updateSucc(LNode*& current, LNode*& f_current, LNode& succ) = 0;

    virtual void evaluate(double& current_p, double& heuristic, double& score) = 0;

private:
    typedef std::multiset<LNode*,CompareHNode> prio_queue;
    double score, heuristic;
    std::vector<LNode*> closedSet;
    std::vector<LNode> twins;
    prio_queue openSet;
};

#endif // LOCAL_PLANNER_STAR_H
