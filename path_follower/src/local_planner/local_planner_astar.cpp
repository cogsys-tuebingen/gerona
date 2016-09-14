/// HEADER
#include <path_follower/local_planner/local_planner_astar.h>

/// PROJECT
#include <path_follower/pathfollower.h>

LocalPlannerAStar::LocalPlannerAStar(PathFollower &controller, tf::Transformer &transformer,
                                     const ros::Duration &update_interval)
    : LocalPlannerClassic(controller,transformer,update_interval),
      LocalPlannerStar(controller,transformer,update_interval)
{

}

double LocalPlannerAStar::G(LNode*& current, std::size_t& index, std::vector<LNode*>& successors,
                            const std::vector<Scorer::Ptr>& scorer, const std::vector<double>& wscorer,
                            double& score){
    double tentative_gScore = current->gScore_ ;
    if(successors[index]->twin_ != nullptr){
        tentative_gScore += Cost(*(successors[index]->twin_), scorer, wscorer, score);
    }else{
        tentative_gScore += Cost(*(successors[index]), scorer, wscorer, score);
    }
    return tentative_gScore;
}

void LocalPlannerAStar::updateSucc(LNode *&current, LNode *&f_current, LNode &succ){
    (void) current;
    (void) f_current;
    (void) succ;
}
