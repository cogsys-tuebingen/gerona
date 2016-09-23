/// HEADER
#include <path_follower/local_planner/local_planner_bfs.h>

/// PROJECT
#include <path_follower/pathfollower.h>

// this planner uses the Breadth-first search algorithm
LocalPlannerBFS::LocalPlannerBFS(PathFollower &follower,
                                 tf::Transformer& transformer,
                                 const ros::Duration& update_interval)
    : LocalPlannerClassic(follower, transformer, update_interval),fifo()
{

}

void LocalPlannerBFS::setInitScores(LNode& wpose, const std::vector<Scorer::Ptr>& scorer,
                               const std::vector<double>& wscorer, double& dis2last){
    (void) wpose;
    (void) scorer;
    (void) wscorer;
    (void) dis2last;
}

void LocalPlannerBFS::initQueue(LNode& root){
    std::queue<LNode*> empty;
    fifo.swap(empty);
    fifo.push(&root);
}

bool LocalPlannerBFS::isQueueEmpty(){
    return fifo.empty();
}

LNode* LocalPlannerBFS::queueFront(){
    return fifo.front();
}

void LocalPlannerBFS::pop(LNode*& current){
    current = fifo.front();
    fifo.pop();
}

void LocalPlannerBFS::push2Closed(LNode*& current){
    (void) current;
}

void LocalPlannerBFS::expandCurrent(LNode*& current, std::size_t& nsize, std::vector<LNode*>& successors,
                                    std::vector<LNode>& nodes, const std::vector<Constraint::Ptr>& constraints,
                                    const std::vector<bool>& fconstraints){
    getSuccessors(current, nsize, successors, nodes, constraints, fconstraints);
}

bool LocalPlannerBFS::processSuccessor(LNode*& succ, LNode*& current,
                                       double& current_p,double& dis2last,
                                       const std::vector<Constraint::Ptr>& constraints,
                                       const std::vector<Scorer::Ptr>& scorer,
                                       const std::vector<bool>& fconstraints,
                                       const std::vector<double>& wscorer){
    (void) current;
    (void) constraints;
    (void) fconstraints;
    fifo.push(succ);
    evaluate(current_p, succ, dis2last, scorer, wscorer);
    return true;
}
