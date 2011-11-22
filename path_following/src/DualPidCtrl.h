#ifndef DUALPIDCTRL_H
#define DUALPIDCTRL_H
#include "Stopwatch.h"
class DualPidCtrl
{
public:
    DualPidCtrl();
    ~DualPidCtrl ();

    /**
      control algorithm
      @param ef error  front axis center to path
      @param er error rear axis center to path
      @param deltaf result steer angle front in rad
      @param deltar result steer angle rear in rad
      */
    bool execute (double ef, double er, float& deltaf, float& deltar);

    /**
      set sampling time (abtastzeit)
      */
    void setTa (double sec) {Ta_=sec;}

    /**
      reset timer and all integrals
      */
    void reset ();

    void configure (double Kp,double delta_max, double e_max, double v , double ta_sec);


protected:
    Stopwatch timer_;
    double Ta_;
    double delta_max_,e_max_, v_;
    double Kp_;

};

#endif // DUALPIDCTRL_H
