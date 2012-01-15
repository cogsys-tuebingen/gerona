/**
   (c) 2012 Karsten Bohlmann bohlmann@gmail.com 

   @author Karsten Bohlmann
   @date   1/11/2012
   @file   SamplingPlanner.h

*/ 

#ifndef SAMPLINGPLANNER_H
#define SAMPLINGPLANNER_H

#include "../ReedsShepp/Curve.h"
#include "../ReedsShepp/CurveGenerator.h"
#include "GoalRegion.h"

namespace lib_path {

class SamplingPlanner
{
public:
    SamplingPlanner(CurveGenerator *rs_generator,MapInfo *map);
    Curve* createPath (const Pose2d& start, GoalRegion *region, int samples_num);
private:
    CurveGenerator *rs_generator_;
    MapInfo *map_;
};

}

#endif // SAMPLINGPLANNER_H
