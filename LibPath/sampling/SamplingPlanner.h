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
class SamplingPlanner
{
public:
    SamplingPlanner(ReedsShepp::CurveGenerator *rs_generator,MapInfo *map);
    ReedsShepp::Curve* createPath (const Pose2d& start, GoalRegion *region, int samples_num);
private:
    ReedsShepp::CurveGenerator *rs_generator_;
    MapInfo *map_;
};

#endif // SAMPLINGPLANNER_H
