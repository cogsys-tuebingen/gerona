/// HEADER
#include <path_follower/local_planner/dis2obst_constraint.h>

double Dis2Obst_Constraint::DIS2O_ = 0.85;

Dis2Obst_Constraint::Dis2Obst_Constraint():
    Constraint()
{

}

Dis2Obst_Constraint::~Dis2Obst_Constraint()
{

}

void Dis2Obst_Constraint::setLimit(double dis2o){
    DIS2O_ = dis2o;
}

bool Dis2Obst_Constraint::isSatisfied(const LNode& point){
    sw.resume();
    double x = point.nop.x - point.x;
    double y = point.nop.y - point.y;
    double a_diff = MathHelper::AngleClamp(point.orientation - std::atan2(y,x));
    double closest_obst = ((3-cos(a_diff)) * point.d2o)/2.0;
    if(closest_obst <= DIS2O_){
        sw.stop();
        return false;
    }
    sw.stop();
    return true;
}
