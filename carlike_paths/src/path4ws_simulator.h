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

using namespace std;
struct Bot4ws
{
  Bot4ws()
    :x_(0.0),y_(0.0),theta_(0.0),beta_(0.0),deltaf_(0.0),deltar_(0.0),l_(0.5)
  {

  }
  void update(float d);
  float x_,y_,theta_, beta_;
  float deltaf_,deltar_;
  float l_;
};

class Path4wsSimulator
{
public:
  Path4wsSimulator();
  void setNumBots (unsigned n);
  void step ();
  void reset ();
  CvArr* getImage() {return map_;}
protected:
  void updateMap();
  void randomizeSteerAngles();
  unsigned n_;
  vector<Bot4ws> bots_;

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
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_;
};

#endif // BOT4WS_SIMULATOR_H
