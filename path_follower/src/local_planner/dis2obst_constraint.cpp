/// HEADER
#include <path_follower/local_planner/dis2obst_constraint.h>

double Dis2Obst_Constraint::DIS2O_ = 0.85;
double Dis2Obst_Constraint::full_d = DIS2O_;

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

void Dis2Obst_Constraint::setVDis(double dis){
    full_d = DIS2O_ + dis;
}

bool Dis2Obst_Constraint::isSatisfied(const LNode& point){
    sw.resume();
    if(point.d2o <= full_d){
        sw.stop();
        return false;
    }
    sw.stop();
    return true;
}
