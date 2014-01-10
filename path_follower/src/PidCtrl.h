#ifndef PIDCTRL_H
#define PIDCTRL_H

#include <utils_general/Stopwatch.h>

class PidCtrl
{
public:
    PidCtrl();
    ~PidCtrl ();

    /**
      control algorithm
      @param ef error  front axis center to path
      @param deltaf result steer angle front in rad
      */
    bool execute (double ef, double& deltaf);

    /**
      set sampling time (abtastzeit)
      */
    void setTa (double sec) {Ta_=sec;}

    /**
      reset timer and all integrals
      */
    void reset ();

    /// Sets Ki to zero
    void configure (double Kp,double delta_max, double e_max, double v , double ta_sec);

    void configure (double Kp, double Ki, double i_max, double delta_max, double e_max, double v, double ta);


protected:
    Stopwatch timer_;
    double Ta_;
    double delta_max_,e_max_, v_;
    double Kp_;
    double Ki_;

    double i_f_;
    double i_max_;
};

#endif // PIDCTRL_H
