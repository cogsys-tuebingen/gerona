/// HEADER
#include <path_follower/local_planner/high_speed/local_planner_bfs.h>

/// PROJECT


// this planner uses the Breadth-first search algorithm
LocalPlannerBFS::LocalPlannerBFS()
    : fifo()
{

}

void LocalPlannerBFS::setInitScores(LNode& wpose, double& dis2last){
    (void) wpose;
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
                                    std::vector<LNode>& nodes){
    getSuccessors(current, nsize, successors, nodes);
}

bool LocalPlannerBFS::processSuccessor(LNode*& succ, LNode*& current,
                                       double& current_p, double& dis2last){
    (void) current;
    fifo.push(succ);
    evaluate(current_p, succ, dis2last);
    return true;
}
