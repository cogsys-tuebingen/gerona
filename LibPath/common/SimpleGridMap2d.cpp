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
    : width_( w ), height_( h ), res_( r ),  origin_( 0, 0 ),
      lowerThres_( 0 ), upperThres_( 80 )
{
    data_.resize( width_*height_ );
}
