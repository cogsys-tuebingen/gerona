/// HEADER
#include <path_follower/local_planner/local_planner_static.h>

/// PROJECT
#include <path_follower/pathfollower.h>

LocalPlannerStatic::LocalPlannerStatic(PathFollower &follower,
                                 tf::Transformer& transformer,
                                 const ros::Duration& update_interval)
    : LocalPlannerClassic(follower, transformer, update_interval)
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

void LocalPlannerStatic::reconfigureTree(LNode*& obj, std::vector<LNode>& nodes, double& best_p,
                                         const std::vector<Constraint::Ptr>& constraints,
                                         const std::vector<Scorer::Ptr>& scorer,
                                         const std::vector<bool>& fconstraints,
                                         const std::vector<double>& wscorer){
    (void) obj;
    (void) nodes;
    (void) best_p;
    (void) constraints;
    (void) scorer;
    (void) fconstraints;
    (void) wscorer;
}
