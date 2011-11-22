/**
  @author karsten bohlmann bohlmann@gmail.com
  @date early 21st century

  */
#include <math.h>
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
}


bool DualPidCtrl::execute(double ef, double er, float &deltaf, float &deltar)
{
    if (timer_.msElapsed()>=Ta_*1000.0) {
        timer_.restart();
        deltaf = Kp_*delta_max_*ef/e_max_;
        deltar = Kp_*delta_max_*er/e_max_;
        return true;
    } else {
        return false;
    }
}


void DualPidCtrl::configure(double Kp, double delta_max, double e_max, double v, double ta)
{
    Kp_=Kp;
    delta_max_=delta_max;
    e_max_ = e_max;
    v_=v;
    Ta_=ta;
    reset ();
}
