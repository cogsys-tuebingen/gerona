/**
  @author karsten bohlmann bohlmann@gmail.com
  @date early 21st century

  */
#include <stdio.h>
#include <math.h>
#include <MathHelper.h>
#include "DualPidCtrl.h"

DualPidCtrl::DualPidCtrl()
{
    configure(0.3,20.0*M_PI/180.0,0.1,0.4,0.1);
}


DualPidCtrl::~DualPidCtrl()
{
    // nothing to do

}


void DualPidCtrl::reset()
{
    timer_.restart();
    i_f_ = i_r_ = 0;
}


bool DualPidCtrl::execute(double ef, double er, double &deltaf, double &deltar)
{
    double d_t = timer_.msElapsed();
    if (d_t >= Ta_*1000.0) {

        i_f_ += d_t*ef/1000.0;
        i_r_ += d_t*er/1000.0;
        i_f_ = min( i_f_, i_max_ );
        i_f_ = max( i_f_, -i_max_ );
        i_r_ = min( i_r_, i_max_ );
        i_r_ = max( i_r_, -i_max_ );

        timer_.restart();
        deltaf = (Kp_ + Ki_*i_f_)*delta_max_*ef/e_max_;
        deltar = (Kp_ + Ki_*i_r_)*delta_max_*er/e_max_;
        deltaf=MathHelper::clamp(deltaf,-delta_max_,+delta_max_);
        deltar = MathHelper::clamp(deltar,-delta_max_,+delta_max_);
        return true;
    } else {
        return false;
    }
}


void DualPidCtrl::configure(double Kp, double delta_max, double e_max, double v, double ta)
{
    Kp_=Kp;
    Ki_ = i_max_ = 0.0;
    delta_max_=delta_max;
    e_max_ = e_max;
    v_=v;
    Ta_=ta;
    reset ();
}

void DualPidCtrl::configure(double Kp, double Ki, double i_max, double delta_max, double e_max, double v, double ta)
{
    configure( Kp, delta_max, e_max, v, ta );
    Ki_ = Ki;
    i_max_ = i_max;
    reset();
}
