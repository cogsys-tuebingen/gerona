/// HEADER
#include <path_follower/local_planner/local_planner_thetastar.h>

/// PROJECT


LocalPlannerThetaStar::LocalPlannerThetaStar(RobotController &controller, PoseTracker &pose_tracker,
                                             const ros::Duration &update_interval)
    : LocalPlannerClassic(controller,pose_tracker,update_interval),
      LocalPlannerStar(controller,pose_tracker,update_interval),alt()
{

}

double LocalPlannerThetaStar::G(LNode*& current, LNode*& succ,
                                const std::vector<Constraint::Ptr>& constraints,
                                const std::vector<Scorer::Ptr>& scorer,
                                const std::vector<bool>& fconstraints,
                                const std::vector<double>& wscorer,
                                double& score){
    double tentative_gScore = current->gScore_ ;
    LNode* succg;
    if(succ->twin_ != nullptr){
        succg = succ->twin_;
    }else{
        succg = succ;
    }
    tentative_gScore += Cost(*(succg), scorer, wscorer, score);
    if(tryForAlternative(succg, constraints, fconstraints)){
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

bool LocalPlannerThetaStar::tryForAlternative(LNode*& s_p, const std::vector<Constraint::Ptr>& constraints,
                                              const std::vector<bool>& fconstraints){
    return createAlternative(s_p,alt,constraints,fconstraints);
}

void LocalPlannerThetaStar::updateSucc(LNode *&current, LNode *&f_current, LNode &succ){
    if(current != f_current){
        succ = alt;
    }
}
