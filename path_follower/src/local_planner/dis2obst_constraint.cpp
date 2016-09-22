/// HEADER
#include <path_follower/local_planner/dis2obst_constraint.h>

double Dis2Obst_Constraint::DIS2O_ = 0.85;
double Dis2Obst_Constraint::vel_ = 0.5;

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

void Dis2Obst_Constraint::setVel(double vel){
    vel_ = vel;
}

bool Dis2Obst_Constraint::isSatisfied(const LNode& point){
    sw.resume();
    //double x = point.nop.x - point.x;
    //double y = point.nop.y - point.y;
    //double a_diff = MathHelper::AngleClamp(std::atan2(y,x) - point.orientation);
    //double closest_obst = ((3-std::cos(a_diff)) * point.d2o)/2.0;
    /*
    double W = 0.61;
    double L = 0.722;
    double D2 = W*W + L*L;
    double D = std::sqrt(D2);
    double c1 = vel_*vel_/(2.0*9.81*0.65);
    double c3 = (L + M_SQRT2*c1 - W)/2.0;
    double co = std::sqrt(0.5 - W/(2*D));
    double co2 = (W*W - L*L)/D2;
    double co4 = (W*W*W*W - 6.0*W*W*L*L + L*L*L*L)/(D2*D2);
    double c2 = (D - L - 2.0*c1*co - c3*(co2 - 1.0)) /(1.0-co4);

    double c_limit = L/2.0 + c1*std::cos(a_diff/2.0) + c2*(1.0 - std::cos(4.0*a_diff))/2.0 + c3*(std::cos(2.0*a_diff) - 1.0)/2.0;

    if(point.d2o <= c_limit){*/
    double c1 = vel_*vel_/(2.0*9.81*1.0);
    if(point.d2o <= DIS2O_ + c1){
    //if(closest_obst <= DIS2O_){
        sw.stop();
        return false;
    }
    sw.stop();
    return true;
}
