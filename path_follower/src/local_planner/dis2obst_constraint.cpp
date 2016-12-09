/// HEADER
#include <path_follower/local_planner/dis2obst_constraint.h>

Dis2Obst_Constraint::Dis2Obst_Constraint():
    Constraint()
{

}

Dis2Obst_Constraint::~Dis2Obst_Constraint()
{

}

void Dis2Obst_Constraint::setParams(double obstacle_threshold){
    threshold = obstacle_threshold;
}

bool Dis2Obst_Constraint::isSatisfied(const LNode& point){
    sw.resume();
    if(point.d2o <= threshold){
        sw.stop();
        return false;
    }
    sw.stop();
    return true;
}
