/// HEADER
#include <path_follower/local_planner/dis2obst_scorer.h>

double Dis2Obst_Scorer::DIS2O_ = 0.85;
double Dis2Obst_Scorer::vdis_ = 0.5;
double Dis2Obst_Scorer::factor_ = 1.0;

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
    vdis_ = dis;
}

void Dis2Obst_Scorer::setFactor(double factor){
    factor_ = factor;
}

double Dis2Obst_Scorer::score(const LNode& point){
    sw.resume();
    double score = std::exp(factor_*((DIS2O_ + vdis_) - point.d2o));
    sw.stop();
    return score;
}
