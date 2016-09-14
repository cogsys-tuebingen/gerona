/// HEADER
#include <path_follower/local_planner/local_planner_thetastar.h>

/// PROJECT
#include <path_follower/pathfollower.h>

LocalPlannerThetaStar::LocalPlannerThetaStar(PathFollower &controller, tf::Transformer &transformer,
                                             const ros::Duration &update_interval)
    : LocalPlannerClassic(controller,transformer,update_interval),
      LocalPlannerStar(controller,transformer,update_interval),alt()
{

}

double LocalPlannerThetaStar::G(LNode*& current, std::size_t& index, std::vector<LNode*>& successors,
                            const std::vector<Scorer::Ptr>& scorer, const std::vector<double>& wscorer,
                            double& score){
    double tentative_gScore = current->gScore_ ;
    LNode* succ;
    if(successors[index]->twin_ != nullptr){
        succ = successors[index]->twin_;
    }else{
        succ = successors[index];
    }
    tentative_gScore += Cost(*(succ), scorer, wscorer, score);
    if(tryForAlternative(succ)){
        double score1;
        double tentative_gScore1 = current->parent_->gScore_ + Cost(alt,scorer,wscorer,score1);
        if(tentative_gScore1 < tentative_gScore){
            score = score1;
            tentative_gScore = tentative_gScore1;
            current = current->parent_;
        }
    }
    return tentative_gScore;
}

bool LocalPlannerThetaStar::tryForAlternative(LNode*& s_p){
    return createAlternative(s_p,alt);
}

void LocalPlannerThetaStar::updateSucc(LNode *&current, LNode *&f_current, LNode &succ){
    if(current != f_current){
        succ = alt;
    }
}
