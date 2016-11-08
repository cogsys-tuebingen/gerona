/// HEADER
#include <path_follower/local_planner/local_planner_reconf.h>

/// PROJECT


LocalPlannerReconf::LocalPlannerReconf()
{

}

void LocalPlannerReconf::initLeaves(LNode& root){
    leaves.clear();
    leaves.push_back(&root);
}

void LocalPlannerReconf::updateLeaves(std::vector<LNode*>& successors, LNode*& current){
    if(!successors.empty()){
        std::vector<LNode*>::iterator current_i = std::find(leaves.begin(),leaves.end(), current);
        leaves.erase(current_i);
    }
}

void LocalPlannerReconf::updateBest(double& current_p, double& best_p, LNode*& obj, LNode*& succ){
    (void) current_p;
    (void) best_p;
    (void) obj;
    (void) succ;
}

void LocalPlannerReconf::addLeaf(LNode*& node){
    if(std::find(leaves.begin(), leaves.end(), node) == leaves.end()){
        leaves.push_back(node);
    }
}

void LocalPlannerReconf::reconfigureTree(LNode*& obj, std::vector<LNode>& nodes, double& best_p){
    if(obj != nullptr){
        if(obj->parent_ != nullptr){
            LNode tParent = *(obj->parent_);
            if(tParent.parent_ != nullptr){
                obj->parent_ = &tParent;
                tParent.parent_ = &nodes[0];
                LNode alternative;
                if(createAlternative(obj,alternative,true)){
                    *obj = alternative;
                }else{
                    obj = nullptr;
                    return;
                }
            }
        }else{
            obj = nullptr;
            return;
        }
    }else{
        std::vector<LNode*> alts;
        for(LNode* leaf: leaves){
            if(leaf->parent_ != nullptr){
                LNode tParent = *(leaf->parent_);
                if(tParent.parent_ != nullptr){
                    leaf->parent_ = &tParent;
                    tParent.parent_ = &nodes[0];
                    LNode alternative;
                    if(createAlternative(leaf,alternative,true)){
                        *leaf = alternative;
                        alts.push_back(leaf);
                    }
                }else{
                    alts.push_back(leaf);
                }
            }
        }
        if(alts.empty()){
            obj = nullptr;
            return;
        }
        for(LNode* altern: alts){
            double current_p = Score(*altern);
            if(current_p < best_p){
                best_p = current_p;
                obj = altern;
            }
        }
    }
}
