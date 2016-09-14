/// HEADER
#include <path_follower/local_planner/level_scorer.h>

Level_Scorer::Level_Scorer():
    Scorer()
{

}

Level_Scorer::~Level_Scorer()
{

}

double Level_Scorer::score(const LNode& point){
    sw.resume();
    double ls = (10.0 - point.level_)/10.0;
    sw.stop();
    return ls;
}
