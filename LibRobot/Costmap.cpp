/*
 *  Costmap.cpp
 *
 *  Created on: Jan 12, 2012
 *      Author: buck <sebastian.buck@student.uni-tuebingen.de>
 */

#include "Costmap.h"

#include <iostream>
#include <cmath>

#ifdef IPP
#include <ipp.h>
#endif

Costmap::Costmap()
  : map_width_(100),map_height_(100)
{
  // create two empty maps, one for obstacles, one for freeway
  blocked_map_=cvCreateMat(map_width_, map_height_, CV_8U);
  dest_blocked_map_=cvCreateMat(map_width_, map_height_, CV_8U);
  free_map_=cvCreateMat(map_width_, map_height_, CV_8U);
  dest_free_map_=cvCreateMat(map_width_, map_height_, CV_8U);

  res_mat_ = cvCreateMatHeader(map_width_,map_height_,CV_8U);

  block_shape_ = cvCreateStructuringElementEx(3, 3, 1, 1,
                                              CV_SHAPE_ELLIPSE, NULL);
  clear_shape_ = cvCreateStructuringElementEx(3, 3, 1, 1,
                                              CV_SHAPE_ELLIPSE, NULL);

#ifdef __IPP_H__
  ippStaticInit();
  std::cout << "IPP loaded" << std::endl;
#else
  std::cout << "IPP not used" << std::endl;
#endif
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


#ifdef __IPP_H__
inline void ippHelper(CvMat* src, CvMat* dest, IplConvKernel *shape, int amount,
                      IppStatus (*fkt) (const Ipp8u*, int, Ipp8u*, int, IppiSize, IppiBorderType, IppiMorphState*),
                      int width, int height) {
  Ipp8u *origin = src->data.ptr;
  Ipp8u *target = dest->data.ptr;

  IppiSize roi = {width, height};

  unsigned dim = shape->nCols * shape->nRows;
  Ipp8u kernel[dim];
  for(unsigned i=0; i<dim;++i){
    kernel[i] = shape->values[i];
  }

  IppiSize kernelSize = {shape->nCols,shape->nRows};
  IppiPoint anchor = {shape->anchorX, shape->anchorY};

  IppiMorphState* ppState;
  ippiMorphologyInitAlloc_8u_C1R( roi.width, kernel, kernelSize, anchor, &ppState);

  fkt( origin, width, target, height, roi, ippBorderRepl, ppState);
  for(unsigned c=1; c<amount; ++c){
    fkt( target, width, target, height, roi, ippBorderRepl, ppState);
  }
  ippiMorphologyFree(ppState);
}
#endif

void Costmap::grow(const std::vector<int8_t> &map, unsigned width, unsigned height, std::vector<int8_t> &result)
{
  grow(map, width, height, result, 4, 4, 10, 1);
}

void Costmap::grow(const std::vector<int8_t>& map, unsigned width, unsigned height,
                   std::vector<int8_t>& result,
                   int dilate, int erode, int threshold, unsigned sample_factor)
{
  if (width!=map_width_||height!=map_height_) {
    // recreate helper matrices with new height/width
    map_width_ = width;
    map_height_ = height;

    refresh();
  }

  // split the map into two separate maps (blocked & free)
  u_char *blocked_data = NULL;
  u_char *free_data = NULL;
  split(map, blocked_data, free_data, threshold);

  CvMat *blocked_map_scaled, *free_map_scaled;
  int inter = CV_INTER_NN;

  int w = width;
  int h = height;

  // calculate the downsampled dimensions
  bool sampled = sample_factor > 1;

  if(sampled){
    w /= sample_factor;
    h /= sample_factor;

    blocked_map_scaled = cvCreateMat(w, h, CV_8U);
    free_map_scaled = cvCreateMat(w, h, CV_8U);

    // TODO: IPP alternative
    //#ifndef __IPP_H__
    cvResize(blocked_map_, blocked_map_scaled, inter);
    cvResize(free_map_, free_map_scaled, inter);
    //#else
    //    ippiResizeSqrPixel_8u_C1R
    //#endif
  }

  if(dilate > 0){
    // dilate obstacles
#ifndef __IPP_H__
    if(sampled){
      cvDilate(blocked_map_scaled, blocked_map_scaled, block_shape_, dilate/sample_factor);
    } else {
      cvDilate(blocked_map_, dest_blocked_map_, block_shape_, dilate);
    }
#else
    if(sampled){
      ippHelper(blocked_map_scaled, blocked_map_scaled, block_shape_, dilate/sample_factor, ippiDilateBorderReplicate_8u_C1R, w, h);
    } else {
      ippHelper(blocked_map_, dest_blocked_map_, block_shape_, dilate, ippiDilateBorderReplicate_8u_C1R, w, h);
    }
#endif
  } else {
    if(!sampled){
      cvCopy(blocked_map_, dest_blocked_map_);
    }
  }


  if(erode > 0){
    // erode unkown
#ifndef __IPP_H__
    if(sampled){
      cvErode(free_map_scaled, free_map_scaled, clear_shape_, erode/sample_factor);
    } else {
      cvErode(free_map_, dest_free_map_, clear_shape_, erode);
    }
#else
    if(sampled){
      ippHelper(free_map_scaled, free_map_scaled, clear_shape_, erode, ippiErodeBorderReplicate_8u_C1R, w, h);
    } else {
      ippHelper(free_map_, dest_free_map_, clear_shape_, erode, ippiErodeBorderReplicate_8u_C1R, w, h);
    }
#endif
  }  else {
    if(!sampled){
      cvCopy(free_map_, dest_free_map_);
    }
  }

  // use the output vector as target array
  res_mat_->data.ptr = (unsigned char*) &result[0];

  // recombine the maps into one map
#ifndef __IPP_H__
  if(sampled){
    cvMax(blocked_map_scaled, free_map_scaled, free_map_scaled);
    cvResize(free_map_scaled, res_mat_, inter);
  } else {
    cvMax(dest_free_map_, dest_blocked_map_, res_mat_);
  }
#else
  if(sampled){
    Ipp8u* s1 = blocked_map_scaled->data.ptr;
    Ipp8u* s2 = free_map_scaled->data.ptr;
    Ipp8u* d = free_map_scaled->data.ptr;
    ippsMaxEvery_8u(s1, s2, d, w * h);

    // TODO: IPP alternative
    cvResize(free_map_scaled, res_mat_, inter);
  } else {
    Ipp8u* s1 = dest_blocked_map_->data.ptr;
    Ipp8u* s2 = dest_free_map_->data.ptr;
    Ipp8u* d = res_mat_->data.ptr;
    ippsMaxEvery_8u(s1, s2, d, w * h);
  }
#endif
}

void Costmap::refresh()
{
  cvReleaseMat(&blocked_map_);
  cvReleaseMat(&free_map_);
  cvReleaseMat(&dest_blocked_map_);
  cvReleaseMat(&dest_free_map_);
  cvReleaseMat(&res_mat_);
  blocked_map_=cvCreateMat(map_width_, map_height_, CV_8U);
  dest_blocked_map_=cvCreateMat(map_width_, map_height_, CV_8U);
  dest_free_map_=cvCreateMat(map_width_, map_height_, CV_8U);
  free_map_=cvCreateMat(map_width_, map_height_, CV_8U);
  res_mat_ = cvCreateMatHeader(map_width_,map_height_,CV_8U);
}

void Costmap::split(const std::vector<int8_t>& map, u_char *blocked_data, u_char *free_data, int threshold)
{
  // get access to the underlying data
  const signed char * map_raw = &map[0];
  blocked_data = blocked_map_->data.ptr;
  free_data = free_map_->data.ptr;
  for(unsigned i = 0; i < map_width_ * map_height_; ++i){
    int8_t val = map_raw[i];

    if(val > threshold){
      blocked_data[i] = val;
      free_data[i] = 0;
    } else {
      blocked_data[i] = 0;
      free_data[i] = val;
    }
  }
}
