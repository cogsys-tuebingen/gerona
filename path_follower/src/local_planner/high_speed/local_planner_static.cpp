/// HEADER
#include <path_follower/local_planner/high_speed/local_planner_static.h>

/// PROJECT


LocalPlannerStatic::LocalPlannerStatic()
{

}

void LocalPlannerStatic::initLeaves(LNode& root){
    (void) root;
}

void LocalPlannerStatic::updateLeaves(std::vector<LNode*>& successors, LNode*& current){
    (void) successors;
    (void) current;
}

void LocalPlannerStatic::updateBest(double& current_p, double& best_p, LNode*& obj, LNode*& succ){
    if(current_p < best_p){
        best_p = current_p;
        obj = succ;
    }
}

void LocalPlannerStatic::addLeaf(LNode*& node){
    (void) node;
}

void LocalPlannerStatic::reconfigureTree(LNode*& obj, std::vector<LNode>& nodes, double& best_p){
    (void) obj;
    (void) nodes;
    (void) best_p;
}
