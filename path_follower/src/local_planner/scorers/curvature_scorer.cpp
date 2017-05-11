/// HEADER
#include <path_follower/local_planner/scorers/curvature_scorer.h>

double Curvature_Scorer::MAX_CURV = 0.0;

Curvature_Scorer::Curvature_Scorer():
    Scorer()
{

}

Curvature_Scorer::~Curvature_Scorer()
{

}

void Curvature_Scorer::setMaxC(double& radius){
    MAX_CURV = 1.0/radius;
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
