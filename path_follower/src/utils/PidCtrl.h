#ifndef PIDCTRL_H
#define PIDCTRL_H

#include <utils_general/Stopwatch.h>

/**
 * @brief Implementing an PI controller (no D in the current implementation).
 * @todo  It would be possible to make this controller multidimentional by using matrizes. This would abolish the need
 *        of the MultiPidWrapper class.
 */
class PidCtrl
{
public:
    PidCtrl();
    ~PidCtrl ();

    /**
     * @brief control algorithm
     * @param ef error  front axis center to path
     * @param deltaf result steer angle front in rad
     */
    bool execute (double ef, double& deltaf);

    /**
     * @brief Set sampling time (abtastzeit)
     */
    void setTa (double sec) {Ta_=sec;}

    /**
     * @brief Reset timer and all integrals
     */
    void reset();

    /**
     * @brief configure. Sets Ki to zero.
     * @param Kp
     * @param delta_max unused!
     * @param e_max     unused!
     * @param v         unused!
     * @param ta_sec
     */
    void configure (double Kp,double delta_max, double e_max, double v , double ta_sec);

    /**
     * @brief configure
     * @param Kp
     * @param Ki
     * @param i_max
     * @param delta_max unused!
     * @param e_max     unused!
     * @param v         unused!
     * @param ta
     */
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
