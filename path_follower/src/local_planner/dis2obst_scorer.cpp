/// HEADER
#include <path_follower/local_planner/dis2obst_scorer.h>

double Dis2Obst_Scorer::factor_ = 1.0;

Dis2Obst_Scorer::Dis2Obst_Scorer():
    Scorer()
{

}

Dis2Obst_Scorer::~Dis2Obst_Scorer()
{

}

void Dis2Obst_Scorer::setFactor(double factor){
    factor_ = factor;
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
    double exponent = factor_*(point.d2o - point.of);
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
