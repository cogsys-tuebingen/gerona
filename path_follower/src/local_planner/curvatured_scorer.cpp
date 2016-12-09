/// HEADER
#include <path_follower/local_planner/curvatured_scorer.h>

double CurvatureD_Scorer::MAX_CURV = 0.0;

CurvatureD_Scorer::CurvatureD_Scorer():
    Scorer()
{

}

CurvatureD_Scorer::~CurvatureD_Scorer()
{

}

void CurvatureD_Scorer::setMaxC(double& radius){
    MAX_CURV = 2.0/radius;
}

double CurvatureD_Scorer::score(const LNode& point){
    sw.resume();
    double diff = 0.0;
    if(point.parent_ != nullptr){
        double c_curv = point.radius_ < std::numeric_limits<double>::infinity() ? 1.0/point.radius_:0.0;
        double p_curv = point.parent_->radius_ < std::numeric_limits<double>::infinity() ? 1.0/point.parent_->radius_:0.0;
        diff = c_curv - p_curv ;
    }
    sw.stop();
    return diff;
}
