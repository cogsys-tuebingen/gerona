/// HEADER
#include <path_follower/local_planner/dis2obst_scorer.h>

double Dis2Obst_Scorer::factor_ = 1.0;

Dis2Obst_Scorer::Dis2Obst_Scorer():
    Scorer()
{

}

Dis2Obst_Scorer::~Dis2Obst_Scorer()
{

}

void Dis2Obst_Scorer::setFactor(double factor){
    factor_ = factor;
}

double Dis2Obst_Scorer::score(const LNode& point){
    sw.resume();
    double score = 0;

    //this should be a parameter
    double obst_min_dist = 4.0;
    if(point.d2o < obst_min_dist){
        double x = point.nop.x - point.x;
        double y = point.nop.y - point.y;
        double orio = std::atan2(y,x);
        double ang_r2o = MathHelper::AngleClamp(orio - point.orientation);

        x = point.npp.x - point.x;
        y = point.npp.y - point.y;
        double orip = std::atan2(y,x);
        double ang_r2p = MathHelper::AngleClamp(orip - point.orientation);

        double ang_p2o = MathHelper::AngleClamp(point.npp.orientation - orio);

        if(std::abs(ang_r2o) <= M_PI/2){

         //costs increase as the angle difference decreases
         //costs for +-pi/2 are zero - robot driving parallelly to obstacle
         double fact_r2o = cos(ang_r2o);
         //costs increase as the angle difference increases
         double fact_r2p = 1.0 - std::abs(std::cos(ang_r2p/2.0));
         //costs increase as the angle difference decreases
         double fact_p2o = 1.0 + std::cos(ang_p2o);
         //costs increase as the distance to the nearest obstacle decreases
         double fact_d2o = point.d2o > 0 ? std::exp(factor_/point.d2o) - 1.0 : std::numeric_limits<double>::infinity();

         score = fact_r2o * fact_r2p * fact_p2o * fact_d2o;

        }
    }

    sw.stop();
    return score;
}
