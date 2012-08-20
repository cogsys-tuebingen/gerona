/**
   (c) 2012 Karsten Bohlmann bohlmann@gmail.com 
   
   @author Karsten Bohlmann
   @date   8/20/2012
   @file   simbot4ws.h
   
*/ 

#ifndef SIMBOT4WS_H
#define SIMBOT4WS_H


struct SimBot4ws
{
  SimBot4ws()
    :x_(0.0),y_(0.0),theta_(0.0),beta_(0.0),deltaf_(0.0),deltar_(0.0),l_(0.5)
  {

  }
  void update(float d);
  float x_,y_,theta_, beta_;
  float deltaf_,deltar_;
  /// wheelbase
  float l_;
};


#endif // SIMBOT4WS_H
