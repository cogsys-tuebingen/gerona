/**
    Copyright (C) 2015 Karsten Bohlmann E&K AUTOMATION GmbH
    All rights reserved.

    @year   2015
    @author Karsten Bohlmann
    @email  bohlmann@gmail.com
    @file   path_simple_pid.h


*/

#ifndef PATH_SIMPLE_PID_H
#define PATH_SIMPLE_PID_H
#include "path_controller.h"
#include <path_follower/utils/parameters.h>
#include <path_follower/utils/pidcontroller.hpp>


class PathSimplePid : public PathController
{
public:
    PathSimplePid();

    virtual bool execute (double dist_error, double angle_error,double velocity, const std::vector<double>& target_u,
                          std::vector<double>& u) ;


private:
    //! Specific parameters of this controller.
    struct ControllerParameters : public Parameters
    {
        P<double> dead_time;
        P<double> l;
        P<float> pid_ta;
        P<float> fwd_pid_kp;
        P<float> fwd_pid_ki;
        P<float> fwd_pid_kd;
        P<float> fwd_cap_steer_deg;
        P<float> bwd_pid_kp;
        P<float> bwd_pid_ki;
        P<float> bwd_pid_kd;
        P<float> bwd_cap_steer_deg;
        P<float> max_steer;
        P<float> weight_dist;
        P<float> weight_angle;
        P<float> clip_angle_error;
        P<float> clip_dist_error;

        ControllerParameters():
            dead_time(this, "~dead_time", 0.1, "Time step that is used by predictPose"),
            l(this, "~wheel_base", 0.98, "Distance between front and rear axes of the robot."),
            pid_ta(this, "~pid/ta", 0.03, "Update interval of the PID controller."),
            fwd_pid_kp(this, "~fwd/pid/kp", 1.0, "Proportional coefficient of the PID controller."),
            fwd_pid_ki(this, "~fwd/pid/ki", 0.001, "Integral coefficient of the PID controller."),
            fwd_pid_kd(this, "~fwd/pid/kd", 0, "Derivative coefficient of the PID controller."),
            fwd_cap_steer_deg(this, "~fwd/cap_steer_deg", 5.0, "Maximum allowed deviation from precomputed angle."),
            bwd_pid_kp(this, "~bwd/pid/kp", 1.0, "Proportional coefficient of the PID controller."),
            bwd_pid_ki(this, "~bwd/pid/ki", 0.001, "Integral coefficient of the PID controller."),
            bwd_pid_kd(this, "~bwd/pid/kd", 0, "Derivative coefficient of the PID controller."),
            bwd_cap_steer_deg(this, "~bwd/cap_steer_deg", 5.0, "Maximum allowed deviation from precomputed angle."),
            max_steer(this, "~max_steer", 1.3, "Maximal allowed steering angle. Higher angles are capped by this value."),
            weight_dist(this, "~pc_weight_dist", 1.0, "Weight of distance error"),
            weight_angle(this, "~pc_weight_angle", 1.0, "Weight of angle error"),
            clip_angle_error(this,"~angle_error_clip",1.0, "Clipping for angle error"),
            clip_dist_error(this,"~dist_error_clip",0.1, "Clipping for distance error")


        {}
    } opt_;
    PidController<1> fwd_steer_pid_;
    PidController<1> bwd_steer_pid_;
    /**
     * @brief clipErrors
     * @param[in,out] angle_error
     * @param[in,out] dist_error
     */
    void clipErrors (double& angle_error, double& dist_error);

    double calcTotalError (double e_dist, double e_angle);
};

#endif // PATH_SIMPLE_PID_H
