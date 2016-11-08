#ifndef LOCAL_PLANNER_BFS_H
#define LOCAL_PLANNER_BFS_H

/// PROJECT
#include <path_follower/local_planner/local_planner_classic.h>
#include <queue>

class LocalPlannerBFS : virtual public LocalPlannerClassic
{
public:
    LocalPlannerBFS();
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

    virtual void evaluate(double& current_p, LNode*& succ, double& dis2last) = 0;
private:
    std::queue<LNode*> fifo;
};

#endif // LOCAL_PLANNER_BFS_H
