/// HEADER
#include <path_follower/local_planner/dis2obst_scorer.h>

double Dis2Obst_Scorer::DIS2O_ = 0.85;
double Dis2Obst_Scorer::factor_ = 1.0;
double Dis2Obst_Scorer::full_d = DIS2O_;

Dis2Obst_Scorer::Dis2Obst_Scorer():
    Scorer()
{

}

Dis2Obst_Scorer::~Dis2Obst_Scorer()
{

}

void Dis2Obst_Scorer::setLimit(double dis2o){
    DIS2O_ = dis2o;
}

void Dis2Obst_Scorer::setVDis(double dis){
    full_d = DIS2O_ + dis;
}

void Dis2Obst_Scorer::setFactor(double factor){
    factor_ = factor;
}

double Dis2Obst_Scorer::computeFrontier(double angle){
    double vdis = full_d - DIS2O_;
    double L = 0.722 + 2.0*vdis;
    double W = 0.61 + 2.0*vdis;
    double beta = std::acos(L/std::sqrt(L*L + W*W));
    double r;
    if(angle <= beta - M_PI || angle > M_PI - beta){
        r = -L/(2.0*std::cos(angle));
    }else if(angle > beta - M_PI && angle <= -beta){
        r = -W/(2.0*std::sin(angle));
    }else if(angle > -beta && angle <= beta){
        r = L/(2.0*std::cos(angle));
    }else if(angle > beta && angle <= M_PI - beta){
        r = W/(2.0*std::sin(angle));
    }
    return r;
}

double Dis2Obst_Scorer::score(const LNode& point){
    sw.resume();
    double score = 0;
    double x = point.nop.x - point.x;
    double y = point.nop.y - point.y;
    double orio = std::atan2(y,x);
    x = point.npp.x - point.x;
    y = point.npp.y - point.y;
    double orip = std::atan2(y,x);
    double adiff1;
    double adiff2 = MathHelper::AngleClamp(orio - orip)/2.0;
    //double exponent = factor_*(point.d2o - full_d);
    double angle = MathHelper::AngleClamp(orio - point.orientation);
    double exponent = factor_*(point.d2o - computeFrontier(angle));
    double factor = std::cos(adiff2)/std::exp(exponent);
    double adiff = MathHelper::AngleClamp(point.npp.orientation - orio)/2.0;
    double co = std::cos(adiff);
    if(co >= 1.0 - co){
        adiff1 = std::abs(MathHelper::AngleClamp(orio - point.orientation));
        score = std::sin(adiff1)*factor;
    }else{
        adiff1 = MathHelper::AngleClamp(point.orientation - orip)/2.0;
        score = std::cos(adiff1)*factor;
    }
    sw.stop();
    return score;
}
