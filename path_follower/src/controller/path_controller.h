/**
    Copyright (C) 2015 Karsten Bohlmann E&K AUTOMATION GmbH
    All rights reserved.

    @year   2015
    @author Karsten Bohlmann
    @email  bohlmann@gmail.com
    @file   path_controller.h


*/

#ifndef PATH_CONTROLLER_H
#define PATH_CONTROLLER_H
#include <vector>
class PathController
{
public:
    virtual bool execute (double dist_error, double angle_error,
                          double velocity, const std::vector<double>& target_u, std::vector<double>& u) = 0;
};

#endif // PATH_CONTROLLER_H
