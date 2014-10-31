/**
  @author karsten bohlmann bohlmann@gmail.com
  @date early 21st century

  */
#include <stdio.h>
#include <math.h>
#include <utils_general/MathHelper.h>
#include <path_follower/utils/PidCtrl.h>

PidCtrl::PidCtrl()
{
    configure(0.3,20.0*M_PI/180.0,0.1,0.4,0.1);
    reset();
}


PidCtrl::~PidCtrl()
{
    // nothing to do

}


void PidCtrl::reset()
{
    timer_.restart();
    i_f_ = 0;
}


bool PidCtrl::execute(double ef, double &deltaf)
{
    double d_t = timer_.msElapsed();
    if (d_t >= Ta_*1000.0) {
        //std::cout << "Stopwatch Elapsed." << std::endl;

        i_f_ += d_t*ef/1000.0;
        i_f_ = min( i_f_, i_max_ );
        i_f_ = max( i_f_, -i_max_ );

        timer_.restart();
//        deltaf = (Kp_ - Ki_*i_f_)*delta_max_*ef/e_max_;
        deltaf = Kp_ * ef + Ki_ * i_f_;
//        deltaf=MathHelper::clamp(deltaf,-delta_max_,+delta_max_);
        return true;
    } else {
        return false;
    }
}


void PidCtrl::configure(double Kp, double delta_max, double e_max, double v, double ta)
{
    configure(Kp, 0.0, 0.0, delta_max, e_max, v, ta);
}

void PidCtrl::configure(double Kp, double Ki, double i_max, double delta_max, double e_max, double v, double ta)
{
    Kp_        = Kp;
    Ki_        = Ki;
    i_max_     = i_max;
    delta_max_ = delta_max; //TODO: unused
    e_max_     = e_max;     //TODO: unused
    v_         = v;         //TODO: unused
    Ta_        = ta;

    reset();
}
