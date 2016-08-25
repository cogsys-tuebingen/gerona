/// HEADER
#include <path_follower/local_planner/dis2start_scorer.h>

Dis2Start_Scorer::Dis2Start_Scorer():
    Scorer()
{

}

Dis2Start_Scorer::~Dis2Start_Scorer()
{

}

double Dis2Start_Scorer::score(const LNode& point){
    sw.resume();
    double score = point.s;
    sw.stop();
    return score;
}
