/**
   @author Andreas Beck-Greinwald
   @date   2012-01-26
   @file   PoseListGoalRegion.cpp
*/

#include "PoseListGoalRegion.h"

using namespace lib_path;

PoseListGoalRegion::PoseListGoalRegion (const utils::PoseListConstPtr &poses) :
		use_gain_ (false),
		counter_ (0),
		poses_ (poses)
{
	// Nothing to do
}


void PoseListGoalRegion::init (unsigned samples_num)
{
	use_gain_ = false;
	counter_ = 0;

	if (poses_ != NULL && poses_->use_gain &&
		poses_->pose.size () == poses_->gain.size ()) {
		use_gain_ = true;
	}
}


bool PoseListGoalRegion::getNextGoal (Pose2d &goal, double& gain)
{
	if (poses_ == NULL || counter_ >= poses_->pose.size ()) {
		return false;
	}

	if (use_gain_) {
		gain = poses_->gain[counter_];
	}

	geometry_msgs::Pose2D pose = poses_->pose[counter_];
	goal.x = pose.x;
	goal.y = pose.y;
	goal.theta = pose.theta;

	counter_++;

	return true;
}

