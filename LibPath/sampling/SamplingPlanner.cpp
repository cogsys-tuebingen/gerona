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
  Pose2d start_map=pos2map(start,*map_);
  if (!map_->isInMap(start_map)) {
    fprintf(stderr,"starting pos %f %f outside map", start.x,start.y);
    return 0;
  }

  region->init(samples_num);
  Pose2d goal;
  Curve* best_curve=0;
  double min_cost = 999999;
  while (region->getNextGoal(goal)) {
    Pose2d goal_map = pos2map(goal, *map_);
    std::cout << "goal map pose "<<goal_map.x << " "<<goal_map.y << std::endl;
    if (!map_->isInMap(goal_map)) {
      std::cout << "invalid goal pose "<<goal.x << " "<<goal.y << std::endl;
      continue;
    }

    Curve* curve=rs_generator_->find_path(start_map,goal_map, map_);
    if (curve && curve->is_valid()) {

      std::cout << "goal:"<< goal_map.x<<" "<<goal_map.y<< "cost: "<<curve->weight() << std::endl;
      std::cout.flush();
      if (curve->weight()<min_cost) {
        if (best_curve!=0) {
          delete best_curve;
        }
        best_curve=curve;
        min_cost=best_curve->weight();
        std::cout << "new best curve with cost:"<<min_cost << std::endl;

std::cout.flush();
      } else {
        delete curve;
        std::cout << "still  best curve has cost:"<<best_curve->weight() << std::endl;
        std::cout.flush();
      }
    } else {
      delete curve;
    }
  }
  return best_curve;

}
