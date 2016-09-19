/// HEADER
#include <path_follower/local_planner/local_planner_star.h>

/// PROJECT
#include <path_follower/pathfollower.h>

// this planner templates the A*/Theta* search algorithms
LocalPlannerStar::LocalPlannerStar(PathFollower &follower,
                                 tf::Transformer& transformer,
                                 const ros::Duration& update_interval)
    : LocalPlannerClassic(follower, transformer, update_interval), score(0.0), heuristic(0.0),
      closedSet(), twins(), openSet()
{

}

void LocalPlannerStar::setInitScores(LNode& wpose, const std::vector<Scorer::Ptr>& scorer,
                   const std::vector<double>& wscorer, double& dis2last){
    wpose.gScore_ = Cost(wpose, scorer, wscorer, score);
    heuristic = Heuristic(wpose, dis2last);
    wpose.fScore_ = f(wpose.gScore_,score,heuristic);
}

void LocalPlannerStar::initQueue(LNode& root){
    closedSet.clear();
    openSet.clear();
    openSet.insert(&root);
}

bool LocalPlannerStar::isQueueEmpty(){
    return openSet.empty();
}

LNode* LocalPlannerStar::queueFront(){
    return *openSet.begin();
}

void LocalPlannerStar::pop(LNode*& current){
    current = *openSet.begin();
    openSet.erase(openSet.begin());
}

void LocalPlannerStar::push2Closed(LNode*& current){
    closedSet.push_back(current);
}

void LocalPlannerStar::expandCurrent(LNode*& current, std::size_t& nsize, std::vector<LNode*>& successors,
                           std::vector<LNode>& nodes, const std::vector<Constraint::Ptr>& constraints,
                           const std::vector<bool>& fconstraints,const std::vector<double>& wscorer){
    twins.clear();
    getSuccessors(current, nsize, successors, nodes, constraints, fconstraints, wscorer, twins, true);
}

bool LocalPlannerStar::processSuccessor(LNode*& succ, LNode*& current,
                                        double& current_p,double& dis2last,
                                        const std::vector<Scorer::Ptr>& scorer,
                                        const std::vector<double>& wscorer){
    if(std::find(closedSet.begin(), closedSet.end(), succ) != closedSet.end()){
        succ->twin_ = nullptr;
        return false;
    }

    LNode* for_current = current;
    double tentative_gScore = G(for_current,succ,scorer,wscorer,score);

    if(tentative_gScore >= succ->gScore_){
        succ->twin_ = nullptr;
        return false;
    }

    if(succ->twin_ != nullptr){
        succ->InfoFromTwin();
    }

    updateSucc(current,for_current,*succ);

    succ->parent_ = for_current;
    succ->gScore_ = tentative_gScore;

    heuristic = Heuristic(*succ, dis2last);

    succ->fScore_ = f(succ->gScore_, score, heuristic);

    prio_queue::const_iterator inOpen = std::find(openSet.begin(), openSet.end(), succ);
    if(inOpen != openSet.end()){
        openSet.erase(inOpen);
    }
    openSet.insert(succ);
    evaluate(current_p, heuristic, score);
    return true;
}
