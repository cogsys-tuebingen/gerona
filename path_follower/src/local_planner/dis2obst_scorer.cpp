/// HEADER
#include <path_follower/local_planner/dis2obst_scorer.h>

double Dis2Obst_Scorer::DIS2O_ = 0.85;
double Dis2Obst_Scorer::vdis_ = 0.5;

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

double Dis2Obst_Scorer::score(const LNode& point){
    sw.resume();
    double score = std::exp((DIS2O_ + vdis_) - point.d2o);
    sw.stop();
    return score;
}
