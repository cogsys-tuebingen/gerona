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

double Dis2Obst_Constraint::computeFrontier(double angle){
    double beta = 0.70151140480816465482;//+-
    double L = 0.722;
    double W = 0.61;
    double r;
    if(angle <= beta - M_PI){
        r = L/(2.0*std::cos(angle + M_PI));
    }else if(angle > beta - M_PI && angle <= -beta){
        r = W/(2.0*std::cos(angle + M_PI_2));
    }else if(angle > -beta && angle <= beta){
        r = L/(2.0*std::cos(angle));
    }else if(angle > beta && angle <= M_PI - beta){
        r = W/(2.0*std::cos(angle - M_PI_2));
    }else if(angle > M_PI - beta){
        r = L/(2.0*std::cos(angle - M_PI));
    }
    double vdis = full_d - DIS2O_;
    r += vdis;
    return r;
}

bool Dis2Obst_Constraint::isSatisfied(const LNode& point){
    sw.resume();
    //if(point.d2o <= full_d){
    double x = point.nop.x - point.x;
    double y = point.nop.y - point.y;
    double angle = MathHelper::AngleClamp(std::atan2(y,x) - point.orientation);
    if(point.d2o <= computeFrontier(angle)){
        sw.stop();
        return false;
    }
    sw.stop();
    return true;
}
