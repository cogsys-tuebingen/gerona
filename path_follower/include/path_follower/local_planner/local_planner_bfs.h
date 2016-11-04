#ifndef LOCAL_PLANNER_BFS_H
#define LOCAL_PLANNER_BFS_H

/// PROJECT
#include <path_follower/local_planner/local_planner_classic.h>
#include <queue>

class LocalPlannerBFS : virtual public LocalPlannerClassic
{
public:
    LocalPlannerBFS(RobotController& controller, PoseTracker& pose_tracker,
                    const ros::Duration& update_interval);
private:
    virtual void setInitScores(LNode& wpose, const std::vector<Scorer::Ptr>& scorer,
                               const std::vector<double>& wscorer, double& dis2last) override;

    virtual void initQueue(LNode& root) override;

    virtual bool isQueueEmpty() override;

    virtual LNode* queueFront() override;

    virtual void pop(LNode*& current) override;

    virtual void push2Closed(LNode*& current) override;

    virtual void expandCurrent(LNode*& current, std::size_t& nsize, std::vector<LNode*>& successors,
                               std::vector<LNode>& nodes, const std::vector<Constraint::Ptr>& constraints,
                               const std::vector<bool>& fconstraints) override;

    virtual bool processSuccessor(LNode*& succ, LNode*& current,
                                  double& current_p,double& dis2last,
                                  const std::vector<Constraint::Ptr>& constraints,
                                  const std::vector<Scorer::Ptr>& scorer,
                                  const std::vector<bool>& fconstraints,
                                  const std::vector<double>& wscorer) override;

    virtual void evaluate(double& current_p, LNode*& succ, double& dis2last,
                          const std::vector<Scorer::Ptr>& scorer,
                          const std::vector<double>& wscorer) = 0;
private:
    std::queue<LNode*> fifo;
};

#endif // LOCAL_PLANNER_BFS_H
