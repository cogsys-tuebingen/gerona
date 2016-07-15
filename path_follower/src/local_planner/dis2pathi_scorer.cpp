/// HEADER
#include <path_follower/local_planner/dis2pathi_scorer.h>

Dis2PathI_Scorer::Dis2PathI_Scorer():
    Scorer()
{

}

Dis2PathI_Scorer::~Dis2PathI_Scorer()
{

}

double Dis2PathI_Scorer::score(const LNode& point){
    sw.resume();
    double sum = point.d2p;
    if(point.parent_ != nullptr){
        LNode* current = point.parent_;
        sum += current->d2p;
        while(current->parent_ != nullptr){
            current = current->parent_;
            sum += current->d2p;
        }
    }
    sw.stop();
    return sum;
}
