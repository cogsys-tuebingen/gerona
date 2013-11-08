/**
   (c) 2012 Karsten Bohlmann bohlmann@gmail.com 
   
   @author Karsten Bohlmann
   @date   7/24/2012
   @file   bot4ws_simulator.cpp
   
*/ 
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include "path4ws_simulator.h"
#include "MathHelper.h"
void SimBot4ws::update(float d)
{
  float nbeta=atan(0.5*(tan(deltaf_)+tan(deltar_)));
  float dtheta = cos(beta_)*(tan(deltaf_)-tan(deltar_))*d/l_;
  x_=x_+d*cos(theta_+dtheta/2.0+(beta_+nbeta)/2.0);
  y_=y_+d*sin(theta_+dtheta/2.0+(beta_+nbeta)/2.0);
  theta_=theta_+dtheta;
/*  while (theta_>2*M_PI) theta_-=2*M_PI;
  while (theta_<2*M_PI) theta_+=2*M_PI;*/
  theta_=MathHelper::AngleClamp(theta_);
  beta_=nbeta;
}


Path4wsSimulator::Path4wsSimulator()
  : cloud_(new pcl::PointCloud<pcl::PointXYZRGB>),use_3d_vis_(true)
{
  setNumBots(1000);
  l_=0.5;
  t_step_=0.1;
  t_=0.0;
  v_=1;
  t_change_=1.0;
  delta_max_=20.0*M_PI/180.0;
  delta_steps_=10;
  delta_step_=delta_max_*2.0/delta_steps_;
  map_width_=map_height_=1000;
  map_res_=0.1;
  map_= cvCreateImage(cvSize(map_width_, map_height_), IPL_DEPTH_8U, 1);
  reset();
}


void Path4wsSimulator::setNumBots(unsigned n)
{
  n_=n;
  bots_.resize(n_);
}


void Path4wsSimulator::reset()
{
  cvSet(map_, cvScalarAll(255));
  t_=0;
  update_count_=0;
  cloud_->clear();
  time_t seconds;
  time(&seconds);
  srand((unsigned int)seconds);

}


void Path4wsSimulator::step()
{
  t_+=t_step_;
  ++update_count_;
  if (update_count_>t_change_/t_step_) {
    update_count_=0;
    randomizeSteerAngles();
  }
  float d=v_*t_step_;
  for (unsigned i=0;i<n_;++i) {
    SimBot4ws& bot=bots_[i];
    bot.update(d);
  }
  if (!use_3d_vis_) {
    updateMap();
  } else {
    updatePointCloud();
  }
}


void Path4wsSimulator::updatePointCloud()
{
  pcl::PointXYZRGB point(255,255,0);
  uint8_t r = 255, g = 255, b = 255;    // Example: Red color
  uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
  float frgb = *reinterpret_cast<float*>(&rgb);
  for (unsigned i=0;i<n_;++i) {
    SimBot4ws& bot=bots_[i];
    if (bot.y_<0) continue;
    point.x=bot.x_;
    point.y=bot.y_;
    point.z=bot.theta_;
    point.rgb=frgb;
    cloud_->push_back(point);
  }
}



void Path4wsSimulator::updateMap()
{
  int offx=map_width_/2;
  int offy=map_height_/2;
  for (unsigned i=0;i<n_;++i) {
    SimBot4ws& bot=bots_[i];
    int x=bot.x_/map_res_+offx;
    int y=bot.y_/map_res_+offy;
    //std::cout<< "x:"<<bot.x_ << " offx:"<<offx<< " y:"<<bot.y_<< " ix:"<<x<<std::endl;
    if (x>=0 && x<map_width_ &&y>=0 && y<map_height_) {
      CvScalar val=cvGet2D(map_,x,y);
      if (val.val[0]>0) {
        val.val[0]=0;
        cvSet2D(map_,x,y,cvScalarAll(0));
      }
    }
  }
}


void Path4wsSimulator::randomizeSteerAngles()
{

  std::cout<< "randomize deltasteps="<<delta_steps_<<" step="<<delta_step_*180.0/M_PI<<std::endl;

  for (unsigned i=0;i<n_;++i) {
    unsigned sf=rand()%(delta_steps_+1);
    unsigned sr=rand()%(delta_steps_+1);
    bots_[i].deltaf_=sf*delta_step_-delta_max_;
    bots_[i].deltar_=sr*delta_step_-delta_max_;
    //bots_[i].deltar_=0;
  }
}

