/**
   (c) 2012 Karsten Bohlmann bohlmann@gmail.com 
   
   @author Karsten Bohlmann
   @date   8/15/2012
   @file   path_field.h
   
*/ 

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
  PathField();

  /**
    returns a code for the current steering configuration of the robot

    */
  unsigned short getSteerConf(float deltaf, float deltar);

  /**
    calculate index of corresponding cell to bot pos
    */
  unsigned int calcCellIdx (SimBot4ws& bot);

  /**
    @return -1 for invalid position
    */
  int calcXyIdx (float x, float y);
  void intersect (const PathField& other_field);
protected:
  float angle_resolution_;
  float dist_resolution_;
  unsigned width_,length_,height_;
  short rotation_angle_;

};

#endif // PATH_FIELD_H
