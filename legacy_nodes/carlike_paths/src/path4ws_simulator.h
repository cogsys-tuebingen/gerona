/**
   (c) 2012 Karsten Bohlmann bohlmann@gmail.com 
   
   @author Karsten Bohlmann
   @date   7/24/2012
   @file   bot4ws_simulator.h
   
*/ 

#ifndef BOT4WS_SIMULATOR_H
#define BOT4WS_SIMULATOR_H
#include <vector>
#include <opencv/cv.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "simbot4ws.h"

class Path4wsSimulator
{
public:
  Path4wsSimulator();
  void setNumBots (unsigned n);
  void step ();
  void reset ();
  CvArr* getImage() {return map_;}
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud() {
    return cloud_;
  }

  void setTimeStep (float t_step) {t_step_=t_step;}
protected:
  void updateMap();
  void updatePointCloud();
  void randomizeSteerAngles();
  unsigned n_;
  vector<SimBot4ws> bots_;

  float l_; // wheelbase
  float t_;
  float t_step_; // time step
  unsigned update_count_;
  float t_change_; // time step between control changes
  float v_; // speed
  float delta_max_; // max steerangle
  float delta_step_; // steering step
  unsigned delta_steps_;
  CvArr *map_;
  float map_res_;
  unsigned map_width_,map_height_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
  bool use_3d_vis_;
};

#endif // BOT4WS_SIMULATOR_H
