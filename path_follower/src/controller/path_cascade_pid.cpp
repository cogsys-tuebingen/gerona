/**
    Copyright (C) 2015 Karsten Bohlmann E&K AUTOMATION GmbH
    All rights reserved.

    @year   2015
    @author Karsten Bohlmann
    @email  bohlmann@gmail.com
    @file   path_simple_pid.cpp


*/

#include "path_cascade_pid.h"

PathCascadePid::PathCascadePid()
{
}


bool PathCascadePid::execute(double dist_error, double angle_error, double velocity, const std::vector<double> &target_u, std::vector<double> &u)
{
    return true;
}
