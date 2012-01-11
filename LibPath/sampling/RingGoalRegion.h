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
class RingGoalRegion : public GoalRegion
{
public:
    RingGoalRegion (const Point2d& center, double radius, double width);
    virtual ~RingGoalRegion ();
    virtual void init (unsigned samples_num);
    virtual bool getNextGoal(Pose2d& goal);
private:
    Point2d center_;
    double  radius_;
    double  width_;
    unsigned samples_num_;
    double  step_angle_rad_;
    unsigned      counter_;
};

#endif // RINGGOALREGION_H
