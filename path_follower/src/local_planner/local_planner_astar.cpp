/// HEADER
#include <path_follower/local_planner/local_planner_astar.h>

/// PROJECT


LocalPlannerAStar::LocalPlannerAStar()
{

}

double LocalPlannerAStar::G(LNode*& current, LNode*& succ,
                            double& score){
    double tentative_gScore = current->gScore_ ;
    if(succ->twin_ != nullptr){
        tentative_gScore += Cost(*(succ->twin_), score);
    }else{
        tentative_gScore += Cost(*(succ), score);
    }
    return tentative_gScore;
}

void LocalPlannerAStar::updateSucc(LNode *&current, LNode *&f_current, LNode &succ){
    (void) current;
    (void) f_current;
    (void) succ;
}
