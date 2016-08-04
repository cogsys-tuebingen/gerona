/// HEADER
#include <path_follower/local_planner/dis2obst_scorer.h>

Dis2Obst_Scorer::Dis2Obst_Scorer():
    Scorer()
{

}

Dis2Obst_Scorer::~Dis2Obst_Scorer()
{

}

double Dis2Obst_Scorer::score(const LNode& point){
    sw.resume();
    double x = point.nop.x - point.x;
    double y = point.nop.y - point.y;
    double a_diff = point.orientation - std::atan2(y,x);
    double closest_obst = ((3-cos(a_diff)) * point.d2o)/2.0;
    double partial = closest_obst * closest_obst;
    sw.stop();
    return 1.0/partial;
}
