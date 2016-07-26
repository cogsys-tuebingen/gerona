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
    double div = std::abs(point.xp*point.ys - point.xs*point.yp);
    if(div != 0.0){
        div /= sqrt(pow(point.xp*point.xp + point.yp*point.yp,3));
    }
    sw.stop();
    return div;
}
