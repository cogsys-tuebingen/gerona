/**
   (c) 2012 Karsten Bohlmann bohlmann@gmail.com 

   @author Karsten Bohlmann
   @date   1/11/2012
   @file   GoalRegion.h

*/ 

#ifndef GOALREGION_H
#define GOALREGION_H
#include "../common/Pose2d.h"

namespace lib_path {

class GoalRegion
{
public:
    virtual ~GoalRegion();
    virtual bool getNextGoal(Pose2d& goal)=0;
    virtual void init (unsigned samples_num)=0;
};

} // namespace "lib_path"

#endif // GOALREGION_H
