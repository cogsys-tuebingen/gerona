/// HEADER
#include <path_follower/local_planner/dis2path_constraint.h>

double Dis2Path_Constraint::SIN_A = std::sin(5.0*M_PI/36.0);
double Dis2Path_Constraint::D_RATE = Dis2Path_Constraint::SIN_A;
double Dis2Path_Constraint::DIS2P_ = 0.3;
double Dis2Path_Constraint::DIS2O_ = 0.85;

Dis2Path_Constraint::Dis2Path_Constraint():
    Constraint(),limit(DIS2P_),level(-1)
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
    D_RATE = step * SIN_A;
}

void Dis2Path_Constraint::setAngle(double angle){
    SIN_A = std::sin(angle);
}

void Dis2Path_Constraint::setLimits(double dis2p, double dis2o){
    DIS2P_ = dis2p;
    DIS2O_ = dis2o;
}

bool Dis2Path_Constraint::isSatisfied(const LNode& point){
    sw.resume();
    if(point.level_ > level && level > -1){
        double tmplimit = limit - D_RATE;
        limit = tmplimit < DIS2P_?DIS2P_:tmplimit;
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
            double adiff = std::abs(MathHelper::AngleClamp(orio - orip));
            if(adiff <= M_PI/6.0){
                double d_diff = (point.d2o - cos(adiff)*point.d2p);
                double tol = (DIS2O_ + DIS2P_) - (limit - DIS2P_) - d_diff;
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
