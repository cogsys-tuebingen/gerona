/**
   (c) 2012 Karsten Bohlmann bohlmann@gmail.com 
   
   @author Karsten Bohlmann
   @date   8/15/2012
   @file   path_field.cpp
   
*/ 

#include "path_field.h"

PathField::PathField(unsigned cells_size_x, unsigned cells_size_y, unsigned cells_size_angle, float xy_resolution,
                     unsigned steer_conf_num, float origin_x, float origin_y)
  :origin_x_(origin_x),origin_y_(origin_y),origin_angle_(0),
   size_x_(cells_size_x),size_y_(cells_size_y), size_angle_(cells_size_angle),
   xy_resolution_(xy_resolution),steer_conf_num_(steer_conf_num),
   grid_size_(-1)
{
  initGrid();
}


PathField::~PathField()
{

}


void PathField::initGrid()
{
  angle_resolution_=2*M_PI/size_angle_;
  grid_row_size_angle_=steer_conf_num_+1;
  grid_row_size_y_=(size_angle_+1)*grid_row_size_angle_;
  grid_row_size_x_=size_y_*grid_row_size_y_;
  grid_size_=size_x_*grid_row_size_x_;
  grid_.resize(grid_size_);
}



void PathField::intersect(const PathField &other_field)
{

}
