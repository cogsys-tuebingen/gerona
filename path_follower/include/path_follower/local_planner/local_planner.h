#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

/// PROJECT
#include <utils_general/MathHelper.h>
#include <utils_general/Stopwatch.h>
#include <path_follower/utils/path.h>
#include <path_follower/utils/path_interpolated.h>
#include <path_follower/local_planner/constraint.h>
#include <path_follower/local_planner/dis2path_constraint.h>
#include <path_follower/local_planner/dis2obst_constraint.h>
#include <path_follower/local_planner/scorer.h>
#include <path_follower/local_planner/dis2start_scorer.h>
#include <path_follower/local_planner/dis2path_scorer.h>
#include <path_follower/local_planner/dis2obst_scorer.h>

class PathFollower;

class LocalPlanner
{
public:
    virtual ~LocalPlanner();

    virtual void setGlobalPath(Path::Ptr path);

    virtual Path::Ptr updateLocalPath(const std::vector<Constraint::Ptr>& constraints,
                                      const std::vector<Scorer::Ptr>& scorer,
                                      const std::vector<bool>& fconstraints,
                                      const std::vector<double>& wscorer) = 0;

    virtual bool isNull() const;

protected:
    LocalPlanner(PathFollower& controller,
                 tf::Transformer &transformer);

protected:
    PathFollower& follower_;
    tf::Transformer &transformer_;

    PathInterpolated global_path_;

    tf::StampedTransform initial_map_to_odom_;
};

#endif // LOCAL_PLANNER_H
