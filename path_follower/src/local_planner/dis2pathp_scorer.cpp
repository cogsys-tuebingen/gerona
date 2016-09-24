/// HEADER
#include <path_follower/local_planner/dis2pathp_scorer.h>

double Dis2PathP_Scorer::MAX_DIS = 0.3;

Dis2PathP_Scorer::Dis2PathP_Scorer():
    Scorer()
{

}

Dis2PathP_Scorer::~Dis2PathP_Scorer()
{

}

void Dis2PathP_Scorer::setMaxD(double& dis){
    MAX_DIS = dis;
}

double Dis2PathP_Scorer::score(const LNode& point){
    sw.resume();
    double p = point.d2p/MAX_DIS;
    p = p > 1.0 ? 1.0 : p;
    p *= p;
    sw.stop();
    return p;
}
