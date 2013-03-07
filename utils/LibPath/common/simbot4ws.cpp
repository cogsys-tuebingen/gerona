/**
   (c) 2012 Karsten Bohlmann bohlmann@gmail.com 
   
   @author Karsten Bohlmann
   @date   7/24/2012
   @file   simbot4ws.cpp
   
*/ 

#include "simbot4ws.h"

SimBot4Ws::SimBot4Ws()
  :x_(0.0),y_(0.0),theta_(0.0),deltaf_(0.0),deltar_(0.0),l_(0.5),v_(0.0),v_max_(1.5)
{
}


void SimBot4Ws::setSpeed(float v)
{
  if (v>v_max_)  v=v_max_;
  if (v<-v_max_) v=-v_max_;
  v_=v;
}

void SimBot4Ws::update(float t,float deltaf, float deltar)
{
  deltaf_=deltaf;
  deltar_=deltar;
  float d=v_*t;
  float nbeta=atan(0.5*(tan(deltaf_)+tan(deltar_)));
  float dtheta = cos(beta)*(tan(deltaf_-tan(deltar_))*d/l_;
  x_=x+d*cos(theta_+dtheta_/2.0+(beta+nbeta)/2.0);
  y_=y+d*sin(theta_+dtheta_/2.0+(beta+nbeta)/2.0);
  theta_=theta_+dtheta;


}
