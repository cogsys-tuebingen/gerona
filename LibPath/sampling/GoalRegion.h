/**
   (c) 2012 Karsten Bohlmann bohlmann@gmail.com 

   @author Karsten Bohlmann
   @date   1/11/2012
   @file   GoalRegion.h

*/ 

#ifndef GOALREGION_H
#define GOALREGION_H
#include "../common/Pose2d.h"
class GoalRegion
{
public:
    virtual ~GoalRegion();
    virtual bool getNextGoal(Pose2d& goal)=0;
    virtual void init (int samples_num)=0;
};

#endif // GOALREGION_H
