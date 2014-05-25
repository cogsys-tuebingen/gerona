#ifndef MULTIPLEPIDWRAPPER_H
#define MULTIPLEPIDWRAPPER_H

#include "PidCtrl.h"
#include <vector>

class MultiplePidWrapper
{
public:
    MultiplePidWrapper(size_t n);

    /**
      set sampling time (abtastzeit)
      */
    void setTa (double sec) {Ta_=sec;}

    PidCtrl& operator[](size_t idx)
    {
        return controllers_[idx];
    }

    void resetAll();

    void configure(size_t idx, double Kp, double delta_max, double e_max, double v, double Ki=0.0, double i_max=0.0);

    bool execute(double errors[], std::vector<double> &result);

private:
    Stopwatch timer_;
    double Ta_;
    std::vector<PidCtrl> controllers_;
};

#endif // MULTIPLEPIDWRAPPER_H
