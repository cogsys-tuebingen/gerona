/*
 *  Costmap.cpp
 *
 *  Created on: Jan 12, 2012
 *      Author: buck <sebastian.buck@student.uni-tuebingen.de>
 */

#include "Costmap.h"

#include <opencv/cv.h>
#include <highgui.h>

#include <iostream>
#include <cmath>

Costmap::Costmap(int threshold, int dilate, int erode)
  : threshold_(threshold), dilate_(dilate), erode_(erode), map_width_(100),map_height_(100)
{
  // create two empty maps, one for obstacles, one for freeway
  blocked_map_=cvCreateMat(map_width_, map_height_, CV_8U);
  dest_blocked_map_=cvCreateMat(map_width_, map_height_, CV_8U);
  free_map_=cvCreateMat(map_width_, map_height_, CV_8U);
  dest_free_map_=cvCreateMat(map_width_, map_height_, CV_8U);

  res_mat_ = cvCreateMatHeader(map_width_,map_height_,CV_8U);
/*  block_shape_ = cvCreateStructuringElementEx(3, 3, 1, 1,
                                              CV_SHAPE_ELLIPSE, NULL);
*/
  block_shape_ = cvCreateStructuringElementEx(3, 3, 1, 1,
                                              CV_SHAPE_ELLIPSE, NULL);
  clear_shape_ = cvCreateStructuringElementEx(3, 3, 1, 1,
                                              CV_SHAPE_ELLIPSE, NULL);


}


Costmap::~Costmap()
{
  cvReleaseStructuringElement(&clear_shape_);
  cvReleaseStructuringElement(&block_shape_);
  cvReleaseMat(&blocked_map_);
  cvReleaseMat(&free_map_);
  cvReleaseMat(&dest_blocked_map_);
  cvReleaseMat(&dest_free_map_);
  cvReleaseMat(&res_mat_);
}


 void Costmap::grow(const std::vector<int8_t>& map, unsigned width, unsigned height,
               std::vector<int8_t>& result)
{
  if (width!=map_width_||height!=map_height_) {
    // recreate helper matrices with new height/width
    cvReleaseMat(&blocked_map_);
    cvReleaseMat(&free_map_);
    cvReleaseMat(&dest_blocked_map_);
    cvReleaseMat(&dest_free_map_);
    cvReleaseMat(&res_mat_);
    map_width_=width;
    map_height_=height;
    blocked_map_=cvCreateMat(map_width_, map_height_, CV_8U);
    dest_blocked_map_=cvCreateMat(map_width_, map_height_, CV_8U);
    dest_free_map_=cvCreateMat(map_width_, map_height_, CV_8U);
    free_map_=cvCreateMat(map_width_, map_height_, CV_8U);
    res_mat_ = cvCreateMatHeader(map_width_,map_height_,CV_8U);
  }

  // get access to the underlying data
  const signed char * map_raw = &map[0];
  u_char *blocked_data = blocked_map_->data.ptr;
  u_char *free_data = free_map_->data.ptr;
  // split the map into two separate maps (blocked & free)

  for(unsigned i = 0; i < width * height; ++i){
    int8_t val = map_raw[i];

    if(val > threshold_){
      blocked_data[i] = val;
      free_data[i] = 0;
    } else {
      blocked_data[i] = 0;
      free_data[i] = val;
    }
  }

  // dilate obstacles and erode freeway


  cvDilate(blocked_map_, dest_blocked_map_, block_shape_, dilate_);
  cvErode(free_map_, dest_free_map_, clear_shape_, erode_);


  res_mat_->data.ptr=(unsigned char*)&result[0];

  // recombine the maps into one map
  cvMax(dest_free_map_, dest_blocked_map_, res_mat_);

  //result.resize(width*height);
  //memcpy(&result[0],free_data,width*height);
  //std::vector<int8_t> result (free_data, free_data + width * height);
  // cleanup


}
