/// HEADER
#include <path_follower/local_planner/dis2path_constraint.h>

double Dis2Path_Constraint::D_RATE = Dis2Path_Constraint::COS_A;

Dis2Path_Constraint::Dis2Path_Constraint():
    Constraint(),limit(0.3),level(-1)
{

}

Dis2Path_Constraint::~Dis2Path_Constraint()
{

}

void Dis2Path_Constraint::setParams(double new_limit, double step){
    if(new_limit > limit){
        limit = new_limit;
        level = 0;
    }
    D_RATE = step * COS_A;
}

bool Dis2Path_Constraint::isSatisfied(const LNode& point){
    sw.resume();
    if(point.level_ > level && level > -1){
        double tmplimit = limit - D_RATE;
        limit = tmplimit < 0.3?0.3:tmplimit;
        level = -1;
    }
    if(point.d2p <= limit){
        sw.stop();
        return true;
    }else{//If an obstacle decrease the space around the path, then the space boundary is extended.
        if(point.d2o < std::numeric_limits<double>::infinity()){
            double x = point.nop.x - point.x;
            double y = point.nop.y - point.y;
            double orio = atan2(y,x);
            x = point.npp.x - point.x;
            y = point.npp.y - point.y;
            double orip = atan2(y,x);
            double adiff = orio - orip;
            adiff += (adiff > M_PI) ? - 2.0*M_PI : (adiff < -M_PI) ? 2.0*M_PI : 0;
            adiff = abs(adiff);
            if(adiff <= M_PI/6){
                double d_diff = (point.d2o - cos(adiff)*point.d2p);
                double tol = 1.15 - (limit - 0.3) - d_diff;
                if(tol > 0.0){
                    limit += tol;
                    level = point.level_;
                    if(point.d2p <= limit){
                        sw.stop();
                        return true;
                    }
                }
            }
        }
    }
    sw.stop();
    return false;
}
