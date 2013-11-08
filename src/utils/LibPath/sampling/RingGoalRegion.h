/**
   (c) 2012 Karsten Bohlmann bohlmann@gmail.com 

   @author Karsten Bohlmann
   @date   1/11/2012
   @file   RingGoalRegion.h

*/ 

#ifndef RINGGOALREGION_H
#define RINGGOALREGION_H

#include "../common/Point2d.h"
#include "GoalRegion.h"

namespace lib_path {

class RingGoalRegion : public GoalRegion
{
public:
   /**
     @param radius radius of ring, if radius is positive sampled orientations are
     counterclockwise, if negative clockwise
     */
    RingGoalRegion (const Point2d& center, double radius, double width);
    virtual ~RingGoalRegion ();
    virtual void init (unsigned samples_num);
    virtual bool getNextGoal(Pose2d& goal, double& gain);
private:
    Point2d center_;
    double  width_;
    unsigned samples_num_;
    double  step_angle_rad_;
    unsigned      counter_;
    double direction_;
    double  radius_;

};

} // namespace "lib_path"

#endif // RINGGOALREGION_H
