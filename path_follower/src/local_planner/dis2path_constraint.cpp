/// HEADER
#include <path_follower/local_planner/dis2path_constraint.h>

double Dis2Path_Constraint::D_RATE = std::sin(5.0*M_PI/36.0);
double Dis2Path_Constraint::DIS2P_ = 0.5;

Dis2Path_Constraint::Dis2Path_Constraint():
    Constraint(),limit(DIS2P_),level(-1)
{

}

Dis2Path_Constraint::~Dis2Path_Constraint()
{

}

void Dis2Path_Constraint::setParams(double new_limit){
    limit = new_limit;
    level = 0;
}

double Dis2Path_Constraint::getLimit(){
    return limit;
}

void Dis2Path_Constraint::setDRate(double d_rate){
    D_RATE = d_rate;
}

void Dis2Path_Constraint::setLimit(double dis2p){
    DIS2P_ = dis2p;
}

bool Dis2Path_Constraint::isSatisfied(const LNode& point){
    sw.resume();

    limit = DIS2P_;

    //this should be a parameter
    double obst_min_dist = 4.0;
    double enhancement_fact = 4.0;
    if(point.d2o <= obst_min_dist){
        limit *= enhancement_fact;
    }
    else{
        limit = DIS2P_;
    }

    if(point.d2p <= limit){
        sw.stop();
        return true;
    }
    sw.stop();
    return false;
}
