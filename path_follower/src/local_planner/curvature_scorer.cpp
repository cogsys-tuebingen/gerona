/// HEADER
#include <path_follower/local_planner/curvature_scorer.h>

Curvature_Scorer::Curvature_Scorer():
    Scorer()
{

}

Curvature_Scorer::~Curvature_Scorer()
{

}

double Curvature_Scorer::score(const LNode& point){
    sw.resume();
    //stuff
    sw.stop();
    return 0.0;
}
