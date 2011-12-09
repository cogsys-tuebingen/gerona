/**************************************************************************
    @project RA Outdoor Robot System
    @author Karsten Bohlmann
    @date 6/1/2011 early 21st century
    (c) Universitaet Tuebingen 2011

**************************************************************************/

#ifndef LASER2DSCAN_H
#define LASER2DSCAN_H
#include "Global.h"

struct Laser2dScan
{
public:
    Laser2dScan();    

    double min_rad_,max_rad_;
    double max_range_;
    DVector ranges_;
    DVector intensities_;
    int     id_;
    double  resolution_;

};

#endif // LASER2DSCAN_H
