/**
 * @file SimpleGridMap2d.cpp
 * @date Jan 2012
 * @author marks
 */

// Project
#include "SimpleGridMap2d.h"

///////////////////////////////////////////////////////////////////////////////
// class SimpleGridMap2d
///////////////////////////////////////////////////////////////////////////////

lib_path::SimpleGridMap2d::SimpleGridMap2d( const unsigned int w, const unsigned int h, const double r )
    :     lower_thres_( 50 ), upper_thres_( 200 )
{
  width_= w ;
  height_= h ;
  res_= r ;
  origin_.x=0;
  origin_.y=0;

    data_.resize( width_*height_ );

}
