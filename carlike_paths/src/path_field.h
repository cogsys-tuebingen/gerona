/**
   (c) 2012 Karsten Bohlmann bohlmann@gmail.com 
   
   @author Karsten Bohlmann
   @date   8/15/2012
   @file   path_field.h
   
*/ 

#ifndef PATH_FIELD_H
#define PATH_FIELD_H
#include <vector>
#include <cmath>
#include "nav_msgs/OccupancyGrid.h"
#include "simbot4ws.h"
struct PathCell
{
  unsigned short  track_id_;
  unsigned int  next_;
  unsigned  short cost_;

};

struct PathXyCell {
  std::vector<PathCell> cells_;

};

class SteerConfig {
public:
  SteerConfig(float max_angle,float max_step);

  unsigned int getSteerConf (float deltaf, float deltar);

private:
  unsigned int getSteerConf(float delta);

  float max_angle_, max_step_;

};


class PathField
{
public:
  /**

    @param origin_x xpos of the leftmost point of map
    @param origin_y ypos of the bottommost point of map
    */
  PathField(unsigned cells_size_x, unsigned cells_size_y, unsigned cells_size_angle, float xy_resolution,
             unsigned steer_conf_num, float origin_x, float origin_y);

  /**
    destructor
    */
  ~PathField ();

  /**
    returns a code for the current steering configuration of the robot

    */
  unsigned int getSteerConf(float deltaf, float deltar)
  {
    return 0;
  }


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

  inline unsigned int calcGridIndex(unsigned int mx, unsigned int my, unsigned int ma, unsigned int msteer)
  {
    return mx*grid_row_size_x_+my*grid_row_size_y_+ma*grid_row_size_angle_+msteer;
  }

  inline bool calcGridIndex (float wx, float wy, float angle, float deltaf, float deltar, unsigned& idx)
  {
    unsigned int mx,my,ma,msteer;
    bool status;
    status= worldToMap2D(wx,wy,mx,my);
    if (!status) return false;
    // normalize angle to [0,2pi]
    while (angle<0) angle+=2*M_PI;
    while (angle>2*M_PI) angle-=2*M_PI;
    ma=angle/angle_resolution_;
    msteer=getSteerConf(deltaf,deltar);
    return true;
  }



protected:

  void initGrid();
  float origin_x_,origin_y_, origin_angle_;
  unsigned size_x_,size_y_,size_angle_;
  float xy_resolution_;
  float angle_resolution_;
  unsigned steer_conf_num_;
  short rotation_angle_;
  int grid_size_,grid_row_size_x_,grid_row_size_y_,grid_row_size_angle_;
  std::vector<PathCell> grid_;
};

#endif // PATH_FIELD_H
