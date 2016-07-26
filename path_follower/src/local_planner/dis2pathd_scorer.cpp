/// HEADER
#include <path_follower/local_planner/dis2pathd_scorer.h>

Dis2PathD_Scorer::Dis2PathD_Scorer():
    Scorer()
{

}

Dis2PathD_Scorer::~Dis2PathD_Scorer()
{

}

double Dis2PathD_Scorer::score(const LNode& point){
    sw.resume();
    double diff = 0.0;
    if(point.parent_ != nullptr){
        diff = std::abs(point.d2p - point.parent_->d2p);
    }
    sw.stop();
    return diff;
}
