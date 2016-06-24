/// HEADER
#include <path_follower/local_planner/dis2pathp_scorer.h>

Dis2PathP_Scorer::Dis2PathP_Scorer():
    Scorer()
{

}

Dis2PathP_Scorer::~Dis2PathP_Scorer()
{

}

double Dis2PathP_Scorer::score(const LNode& point){
    sw.resume();
    double p = point.d2p;
    sw.stop();
    return p;
}
