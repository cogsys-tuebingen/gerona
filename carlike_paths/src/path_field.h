/**
   (c) 2012 Karsten Bohlmann bohlmann@gmail.com 
   
   @author Karsten Bohlmann
   @date   8/15/2012
   @file   path_field.h
   
*/ 
/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#ifndef PATH_FIELD_H
#define PATH_FIELD_H
#include "simbot4ws.h"
struct PathCell
{
  unsigned short steer_conf_;
  unsigned short  count_;
  unsigned int  next_;
  unsigned  int cost_;

};

struct PathXyCell {
  vector<PathCell> cells_;

};

class PathField
{
public:
  /**

    @param origin_x xpos of the leftmost point of map
    @param origin_y ypos of the bottommost point of map
    */
  PathField(unsigned cells_size_x, unsigned cells_size_y, unsigned cells_size_z,
            double xy_resolution, double angle_resolution, double origin_x, double origin_y);

  /**
    returns a code for the current steering configuration of the robot

    */
  unsigned short getSteerConf(float deltaf, float deltar);


  void intersect (const PathField& other_field);


  inline bool worldToMap2D(float wx, float wy, unsigned int& mx, unsigned int& my){
    if(wx < origin_x_ || wy < origin_y_ )
      return false;

    mx = (unsigned) ((wx - origin_x_) / xy_resolution_);
    my = (unsigned) ((wy - origin_y_) / xy_resolution_);

    if(mx < size_x_ && my < size_y_ )
      return true;

    return false;
  }

  inline void mapToWorld2D(unsigned int mx, unsigned int my, float& wx, float& wy){
    //returns the center point of the cell
    wx = origin_x_ + (mx + 0.5) * xy_resolution_;
    wy = origin_y_ + (my + 0.5) * xy_resolution_;
  }

  inline void calcGridIndex(unsigned int mx, unsigned int my, unsigned int ma, unsigned int msteer)
  {


  }

protected:

  void initGrid();
  unsigned size_x_,size_y_,size_angle_;
  float xy_resolution_;
  float angle_resolution_;
  float origin_x_,origin_y_, origin_angle_;
  short rotation_angle_;
  PathCell *grid_;
};

#endif // PATH_FIELD_H
