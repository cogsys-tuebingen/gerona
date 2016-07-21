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
    double x = point.nop.x - point.x;
    double y = point.nop.y - point.y;
    double a_diff = point.orientation - atan2(y,x);
    double closest_obst = ((3-cos(a_diff)) * point.d2o)/2.0;
    if(closest_obst <= 0.85){
        sw.stop();
        return false;
    }
    sw.stop();
    return true;
}
