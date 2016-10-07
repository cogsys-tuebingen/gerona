/// HEADER
#include <path_follower/local_planner/dis2obst_constraint.h>

Dis2Obst_Constraint::Dis2Obst_Constraint():
    Constraint()
{

}

Dis2Obst_Constraint::~Dis2Obst_Constraint()
{

}

bool Dis2Obst_Constraint::isSatisfied(const LNode& point){
    sw.resume();
    if(point.d2o <= point.of){
        sw.stop();
        return false;
    }
    sw.stop();
    return true;
}
