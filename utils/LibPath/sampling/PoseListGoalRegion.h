/**
   @author Andreas Beck-Greinwald
   @date   2012-01-26
   @file   PoseListGoalRegion.cpp
*/
#ifndef POSELISTGOALREGION_H_
#define POSELISTGOALREGION_H_

#include "../common/Point2d.h"
#include "GoalRegion.h"
#include <utils/PoseList.h>

namespace lib_path {

class PoseListGoalRegion : public GoalRegion
{
public:

	PoseListGoalRegion(const utils::PoseListConstPtr &poses);

	virtual void init (unsigned samples_num);
	virtual bool getNextGoal (Pose2d &goal, double& gain);

private:

	bool use_gain_;
	unsigned counter_;
	utils::PoseList::ConstPtr poses_;

};

} // Namespace lib_path

#endif /* POSELISTGOALREGION_H_ */
