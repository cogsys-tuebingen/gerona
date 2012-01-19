/**
 * @file Costmap2dWrapper.cpp
 * @date Jan 2012
 * @author marks
 */

// Project
#include "Costmap2dWrapper.h"

Costmap2dWrapper::Costmap2dWrapper( costmap_2d::Costmap2D *map )
    : costmap_( map ),
      lower_thres_( 20 ),
      upper_thres_( 200 )
{ /* Nothing to do */ }
