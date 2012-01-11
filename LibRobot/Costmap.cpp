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
  : threshold_(threshold), dilate_(dilate), erode_(erode)
{
}

std::vector<int8_t> Costmap::grow(std::vector<int8_t> map, int width, int height)
{
  // create two empty maps, one for obstacles, one for freeway
  CvMat * blocked = cvCreateMat(width, height, CV_8U);
  CvMat * free = cvCreateMat(width, height, CV_8U);

  // get access to the underlying data
  signed char * map_raw = &map[0];
  u_char *blocked_data = blocked->data.ptr;
  u_char *free_data = free->data.ptr;

  // split the map into two separate maps (blocked & free)
  for(int i = 0; i < width * height; ++i){
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
  IplConvKernel * circle_shape = cvCreateStructuringElementEx(3, 3, 1, 1, CV_SHAPE_ELLIPSE, NULL);
  cvDilate(blocked, blocked, circle_shape, dilate_);
  cvErode(free, free, circle_shape, erode_);

  // recombine the maps into one map, use map "free" as result
  cvMax(free, blocked, free);
  std::vector<int8_t> result (free_data, free_data + width * height);

  // cleanup
  cvReleaseStructuringElement(&circle_shape);
  cvReleaseMat(&blocked);
  cvReleaseMat(&free);

  return result;
}
