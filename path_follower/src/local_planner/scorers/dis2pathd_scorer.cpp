/// HEADER
#include <path_follower/local_planner/scorers/dis2pathd_scorer.h>

double Dis2PathD_Scorer::MAX_DIS = 0.3;

Dis2PathD_Scorer::Dis2PathD_Scorer():
    Scorer()
{

}

Dis2PathD_Scorer::~Dis2PathD_Scorer()
{

}

void Dis2PathD_Scorer::setMaxD(double& dis){
    MAX_DIS = dis;
}

double Dis2PathD_Scorer::score(const LNode& point){
    sw.resume();
    double diff = 0.0;
    if(point.parent_ != nullptr){
        diff = (point.d2p - point.parent_->d2p);
    }
    sw.stop();
    return diff;
}
