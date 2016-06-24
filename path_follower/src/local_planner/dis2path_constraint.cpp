/// HEADER
#include <path_follower/local_planner/dis2path_constraint.h>

Dis2Path_Constraint::Dis2Path_Constraint():
    Constraint()
{

}

Dis2Path_Constraint::~Dis2Path_Constraint()
{

}

bool Dis2Path_Constraint::isSatisfied(const LNode& point){
    sw.resume();
    if(point.d2p < 0.3){
        sw.stop();
        return true;
    }
    sw.stop();
    return false;
}
