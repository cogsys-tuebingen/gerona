/**
   (c) 2012 Karsten Bohlmann bohlmann@gmail.com 
   
   @author Karsten Bohlmann
   @date   8/15/2012
   @file   path_field.h
   
*/ 

#ifndef PATH_FIELD_H
#define PATH_FIELD_H

struct PathCell
{
  short steer_astep_;
};


class PathField
{
public:
  PathField();

  void intersect (const PathField& other_field);
protected:
  float angle_resolution_;
  float dist_resolution_;
  unsigned width_,length_,height_;
  short rotation_angle_;
};

#endif // PATH_FIELD_H
