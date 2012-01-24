/**
   (c) 2012 Karsten Bohlmann bohlmann@gmail.com 

   @author Karsten Bohlmann
   @date   1/11/2012
   @file   SamplingPlanner.cpp

*/ 
#include <float.h>
#include <stdio.h>
#include "SamplingPlanner.h"

using namespace lib_path;

SamplingPlanner::SamplingPlanner(CurveGenerator *rs_generator,
                                 GridMap2d* map)
  :rs_generator_(rs_generator),map_(map)
{

}


Curve* SamplingPlanner::createPath(const Pose2d& start, GoalRegion *region, int samples_num)
{
  // TODO: pos2map macht das selbe wie map_->point2cell, nur double statt integers,
  // besser pos2map abschaffen und in GridMap2d mit eingliedern.
  Pose2d start_map=pos2map(start,*map_);
  if (!map_->isInMap(start)) {
    fprintf(stderr,"starting pos %f %f outside map", start.x,start.y);
    return 0;
  }

  region->init(samples_num);

  Pose2d goal;
  Curve* best_curve=0;
  double gain = 1.0;
  double min_cost = 999999;
  while (region->getNextGoal(goal, gain)) {
    Pose2d goal_map = pos2map(goal, *map_);
    std::cout << "goal map pose "<<goal_map.x << " "<<goal_map.y << std::endl;
    if (!map_->isInMap(goal)) {
      std::cout << "invalid goal pose "<<goal.x << " "<<goal.y << std::endl;
      continue;
    }

    Curve* curve=rs_generator_->find_path(start_map,goal_map, map_);
    if (curve && curve->is_valid()) {

      std::cout << "goal:" << goal_map.x << " " << goal_map.y << "cost: " << curve->weight() << std::endl;
      std::cout.flush();

      double actual_cost = gain*curve->weight();

      if (actual_cost<min_cost) {
        if (best_curve!=0) {
          delete best_curve;
        }

        best_curve=curve;
        min_cost=actual_cost;

        std::cout << "new best curve with cost:" << min_cost << std::endl;
        std::cout.flush();
      } else {
        delete curve;
        std::cout << "still  best curve has cost:" << min_cost << std::endl;
        std::cout.flush();
      }
    } else if (curve) {
      delete curve;
    }
  }
  return best_curve;
}
