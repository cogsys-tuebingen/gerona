/**
   (c) 2012 Karsten Bohlmann bohlmann@gmail.com 
   
   @author Karsten Bohlmann
   @date   8/15/2012
   @file   path_field.cpp
   
*/ 

#include "path_field.h"

PathField::PathField(unsigned cells_size_x, unsigned cells_size_y, unsigned cells_size_z, double xy_resolution,
                     double angle_resolution, double origin_x, double origin_y)
  :size_x_(cells_size_x),size_y_(cells_size_y),size_angle_(2*M_PI),xy_resolution_(xy_resolution),
    angle_resolution_(angle_resolution),origin_x_(origin_x),origin_y_(origin_y)
{
}


void PathField::initMap()
{

}
