#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

/// PROJECT
#include <cslibs_utils/MathHelper.h>
#include <cslibs_utils/Stopwatch.h>
#include <path_follower/utils/path.h>
#include <path_follower/utils/path_interpolated.h>
#include <path_follower/parameters/local_planner_parameters.h>
#include <path_follower/local_planner/constraint.h>
#include <path_follower/local_planner/dis2path_constraint.h>
#include <path_follower/local_planner/dis2obst_constraint.h>
#include <path_follower/local_planner/scorer.h>
#include <path_follower/local_planner/dis2pathd_scorer.h>
#include <path_follower/local_planner/dis2pathp_scorer.h>
#include <path_follower/local_planner/dis2obst_scorer.h>
#include <path_follower/local_planner/level_scorer.h>
#include <path_follower/local_planner/curvature_scorer.h>
#include <path_follower/local_planner/curvatured_scorer.h>

class PathFollower;
class PoseTracker;
class ObstacleCloud;

class LocalPlanner
{
public:
    virtual ~LocalPlanner();

    virtual void init(RobotController *controller, PoseTracker *pose_tracker,
                      const LocalPlannerParameters &opt);

    virtual void setGlobalPath(Path::Ptr path);

    virtual void setVelocity(geometry_msgs::Twist::_linear_type vector) = 0;

    virtual void setVelocity(double velocity) = 0;

    virtual void reset();

    virtual Path::Ptr updateLocalPath(Path::Ptr& wlp) = 0;

    virtual bool isNull() const;

    void setObstacleCloud(const std::shared_ptr<ObstacleCloud const> &msg);

    void addConstraint(Constraint::Ptr constraint);
    void addScorer(Scorer::Ptr scorer, double weight);

protected:
    LocalPlanner();

    virtual void setParams(const LocalPlannerParameters& opt) = 0;

protected:
    RobotController* controller_;
    PoseTracker* pose_tracker_;
    tf::Transformer* transformer_;

    const LocalPlannerParameters* opt_;

    std::vector<Constraint::Ptr> constraints;
    std::vector<Scorer::Ptr> scorers;

    PathInterpolated global_path_;

    ros::Duration update_interval_;

    std::shared_ptr<ObstacleCloud const> obstacle_cloud_, last_obstacle_cloud_;
};

#endif // LOCAL_PLANNER_H
