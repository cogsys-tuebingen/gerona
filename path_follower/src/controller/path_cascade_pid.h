/**
    Copyright (C) 2015 Karsten Bohlmann E&K AUTOMATION GmbH
    All rights reserved.

    @year   2015
    @author Karsten Bohlmann
    @email  bohlmann@gmail.com
    @file   path_simple_pid.h


*/

#ifndef PATH_CASCADE_PID_H
#define PATH_CASCADE_PID_H
#include "path_controller.h"
#include <path_follower/utils/parameters.h>
#include <path_follower/utils/pidcontroller.hpp>

class PathCascadePid : public PathController
{
public:
    PathCascadePid();

    virtual bool execute (double dist_error, double angle_error,double velocity, const std::vector<double>& target_u,
                          std::vector<double>& u) ;
private:
    //! Specific parameters of this controller.
    struct ControllerParameters : public Parameters
    {
        P<float> pid_ta;
        P<float> fwd_csca_kp;
        P<float> fwd_csca_ki;
        P<float> fwd_csca_kd;
        P<float> fwd_cscd_kp;
        P<float> fwd_cscd_ki;
        P<float> fwd_cscd_kd;
        P<float> bwd_csca_kp;
        P<float> bwd_csca_ki;
        P<float> bwd_csca_kd;
        P<float> bwd_cscd_kp;
        P<float> bwd_cscd_ki;
        P<float> bwd_cscd_kd;

        P<float> fwd_cap_steer_deg;

        P<float> bwd_cap_steer_deg;
        P<float> clip_dist_error;
        P<float> clip_angle_error;



        ControllerParameters():
            pid_ta(this, "~pid/ta", 0.01, "Update interval of the PID controller."),
            fwd_csca_kp(this, "~fwd/csca/kp", 1.0, "Proportional coefficient of the PID controller."),
            fwd_csca_ki(this, "~fwd/csca/ki", 0.001, "Integral coefficient of the PID controller."),
            fwd_csca_kd(this, "~fwd/csca/kd", 0, "Derivative coefficient of the PID controller."),
            fwd_cscd_kp(this, "~fwd/cscd/kp", 1.0, "Proportional coefficient of the PID controller."),
            fwd_cscd_ki(this, "~fwd/cscd/ki", 0.001, "Integral coefficient of the PID controller."),
            fwd_cscd_kd(this, "~fwd/cscd/kd", 0, "Derivative coefficient of the PID controller."),
            bwd_csca_kp(this, "~bwd/csca/kp", 1.0, "Proportional coefficient of the PID controller."),
            bwd_csca_ki(this, "~bwd/csca/ki", 0.001, "Integral coefficient of the PID controller."),
            bwd_csca_kd(this, "~bwd/csca/kd", 0, "Derivative coefficient of the PID controller."),
            bwd_cscd_kp(this, "~bwd/cscd/kp", 1.0, "Proportional coefficient of the PID controller."),
            bwd_cscd_ki(this, "~bwd/cscd/ki", 0.001, "Integral coefficient of the PID controller."),
            bwd_cscd_kd(this, "~bwd/cscd/kd", 0, "Derivative coefficient of the PID controller."),

            fwd_cap_steer_deg(this, "~fwd/cap_steer_deg", 5.0, "Maximum allowed deviation from precomputed angle."),
            bwd_cap_steer_deg(this, "~bwd/cap_steer_deg", 5.0, "Maximum allowed deviation from precomputed angle."),
            clip_dist_error(this,"~dist_error_clip",1.0, "Clipping for distance error"),
            clip_angle_error(this,"~angle_error_clip",1.0, "Clipping for angle error")



        {}
    } opt_;
    PidController<1> fwd_angle_pid_, fwd_dist_pid_;
    PidController<1> bwd_angle_pid_, bwd_dist_pid_;

    void clipErrors (double& angle_error, double& dist_error);

};

#endif // PATH_CASCADE_PID_H
