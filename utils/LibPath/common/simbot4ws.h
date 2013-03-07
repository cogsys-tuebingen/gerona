/**
   (c) 2012 Karsten Bohlmann bohlmann@gmail.com 
   
   @author Karsten Bohlmann
   @date   7/24/2012
   @file   simbot4ws.h
   
*/ 

#ifndef SIMBOT4WS_H
#define SIMBOT4WS_H
#include <Eigen/Core>
using namespace Eigen;
class SimBot4Ws
{
public:
  SimBot4Ws();

  void update (float t,float deltaf, float deltar);
  void setSpeed (float v);
protected:
  float x_,y_,theta_;
  float deltaf_,deltar_;
  float l_;
  float v_;
  float v_max_;

};

#endif // SIMBOT4WS_H
