/**
   (c) 2012 Karsten Bohlmann bohlmann@gmail.com 

   @author Karsten Bohlmann
   @date   1/11/2012
   @file   SamplingPlanner.cpp

*/ 
#include <float.h>
#include <stdio.h>
#include "SamplingPlanner.h"

SamplingPlanner::SamplingPlanner(ReedsShepp::CurveGenerator *rs_generator,
                                 MapInfo* map)
  :rs_generator_(rs_generator),map_(map)
{

}


ReedsShepp::Curve* SamplingPlanner::createPath(const Pose2d& start, GoalRegion *region)
{
  Pose2d start_map=pos2map(start,*map_);
  if (!map_->isPosValid(start_map)) {
    fprintf(stderr,"starting pos %f %f outside map", start.x,start.y);
    return 0;
  }

  region->init(10);
  Pose2d goal;
  ReedsShepp::Curve* best_curve=0;
  double min_cost = DBL_MAX;
  while (region->getNextGoal(goal)) {
    Pose2d goal_map = pos2map(goal, *map_);
    if (!map_->isPosValid(goal)) {
      std::cout << "invalid goal pose "<<goal.x << " "<<goal.y << std::endl;
      continue;
    }
    ReedsShepp::Curve* curve=rs_generator_->find_path(start_map,goal_map,map_);
    if (curve->is_valid()) {
      std::cout << "goal:"<< goal.x<<" "<<goal.y<< "cost: "<<curve->weight() << std::endl;
      if (curve->weight()<min_cost) {
        if (best_curve!=0) {
          delete best_curve;
        }
        best_curve=curve;
        min_cost=curve->weight();
      } else {
        delete curve;
      }
    }
  }
  return best_curve;

}
