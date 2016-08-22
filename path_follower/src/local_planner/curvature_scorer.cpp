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
    if(point.radius_ < std::numeric_limits<double>::infinity()){
        double div = std::abs(1.0/point.radius_);
        sw.stop();
        return div;
    }
    sw.stop();
    return 0.0;
}
