/**************************************************************************
    @project RA Outdoor Robot System
    @author Karsten Bohlmann
    @date 6/1/2011 early 21st century
    (c) Universitaet Tuebingen 2011

**************************************************************************/
#include <math.h>
#include "Laser2dScan.h"

Laser2dScan::Laser2dScan()
  :min_rad_(-M_PI/2.0),max_rad_(M_PI/2.0)
{
  resolution_=1;
}
