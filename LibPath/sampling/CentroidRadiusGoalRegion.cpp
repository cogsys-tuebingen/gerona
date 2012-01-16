/**
   @author Andreas Beck-Greinwald
   @date   2012-01-15
   @file   CentroidRadiusGoalRegion.cpp
*/ 

#include <math.h>
#include "CentroidRadiusGoalRegion.h"

using namespace lib_path;

CentroidRadiusGoalRegion::CentroidRadiusGoalRegion (const Point2d &center, double radius)
  : center_ (center), radius_ (radius)
{
  init (10);
}


CentroidRadiusGoalRegion::~CentroidRadiusGoalRegion ()
{
  // nothing to do
}


void CentroidRadiusGoalRegion::init (unsigned samples_num)
{
  counter_ = 0;
  samples_num_ = samples_num;

  theta_ = MathHelper::Angle (Vector2d (center_.x, center_.y)) - 0.5 * M_PI_2;
  theta_step_ = M_PI_2 / (samples_num_-1);
}


bool CentroidRadiusGoalRegion::getNextGoal (Pose2d &goal)
{
  if (counter_ >= samples_num_)
    return false;

  goal.x = center_.x - radius_ * cos (theta_);
  goal.y = center_.y - radius_ * sin (theta_);
  goal.theta = theta_;

  counter_++;
  theta_ = MathHelper::AngleClamp (theta_ + theta_step_);

  return true;
}

