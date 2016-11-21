/// HEADER
#include <path_follower/local_planner/local_planner_thetastar.h>

/// PROJECT


LocalPlannerThetaStar::LocalPlannerThetaStar()
    : alt()
{

}

double LocalPlannerThetaStar::G(LNode*& current, LNode*& succ,
                                double& score){
    double tentative_gScore = current->gScore_ ;
    LNode* succg;
    if(succ->twin_ != nullptr){
        succg = succ->twin_;
    }else{
        succg = succ;
    }
    tentative_gScore += Cost(*(succg), score);
    if(tryForAlternative(succg)){
        double score1;
        double tentative_gScore1 = current->parent_->gScore_ + Cost(alt,score1);
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
