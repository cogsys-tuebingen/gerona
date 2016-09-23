/**
    Copyright (C) 2015 Karsten Bohlmann E&K AUTOMATION GmbH
    All rights reserved.

    @year   2015
    @author Karsten Bohlmann
    @email  bohlmann@gmail.com
    @file   path_simple_pid.cpp


*/

#include "path_simple_pid.h"

PathSimplePid::PathSimplePid()
{

    fwd_steer_pid_ = PidController<1>(opt_.fwd_pid_kp(), opt_.fwd_pid_ki(), opt_.fwd_pid_kd(), opt_.pid_ta());
    bwd_steer_pid_ = PidController<1>(opt_.bwd_pid_kp(), opt_.bwd_pid_ki(), opt_.bwd_pid_kd(), opt_.pid_ta());

}


bool PathSimplePid::execute(double dist_error, double angle_error, double velocity, const std::vector<double> &target_u, std::vector<double> &u)
{
    clipErrors(angle_error,dist_error);
    double e = calcTotalError(dist_error,angle_error);
    double v = fabs(velocity);
    if(v < 1e-3) {
        e = 0;
    } else if (v>=1.0) {
        // reduce error for high speeds
        e = e / v;
    }
    float u_val = 0.0f;
    bool do_control;
    if (velocity>1e-3) {
        do_control = fwd_steer_pid_.execute(e, &u_val);
    } else if(velocity < -1e-3) {
        do_control = bwd_steer_pid_.execute(e, &u_val);

    }
    u.resize(1);
    u[0]=u_val;
    return do_control;
}



double PathSimplePid::calcTotalError (double e_dist, double e_angle)
{


    // should be scaled depending on velocity
    // 5 degrees angular error = 0.09
    return ((opt_.weight_dist()*  e_dist +  opt_.weight_angle()*e_angle)/(opt_.weight_dist()+opt_.weight_angle()));

}


void PathSimplePid::clipErrors(double &angle_error, double &dist_error)
{
    if (angle_error>opt_.clip_angle_error()) {
        angle_error = opt_.clip_angle_error();
    } else if (angle_error<-opt_.clip_angle_error()) {
        angle_error = -opt_.clip_angle_error();

    }
    if (dist_error>opt_.clip_dist_error()) {
        dist_error = opt_.clip_dist_error();
    } else if (dist_error<-opt_.clip_dist_error()) {
        dist_error = -opt_.clip_dist_error();

    }



}
