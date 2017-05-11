#ifndef ABSTRACT_LOCAL_PLANNER_H
#define ABSTRACT_LOCAL_PLANNER_H

/// PROJECT
#include <cslibs_utils/MathHelper.h>
#include <cslibs_utils/Stopwatch.h>
#include <path_follower/utils/path.h>
#include <path_follower/utils/path_interpolated.h>
#include <path_follower/parameters/local_planner_parameters.h>
#include <path_follower/local_planner/constraint.h>
#include <path_follower/local_planner/constraints/dis2path_constraint.h>
#include <path_follower/local_planner/constraints/dis2obst_constraint.h>
#include <path_follower/local_planner/scorer.h>
#include <path_follower/local_planner/scorers/dis2pathd_scorer.h>
#include <path_follower/local_planner/scorers/dis2pathp_scorer.h>
#include <path_follower/local_planner/scorers/dis2obst_scorer.h>
#include <path_follower/local_planner/scorers/level_scorer.h>
#include <path_follower/local_planner/scorers/curvature_scorer.h>
#include <path_follower/local_planner/scorers/curvatured_scorer.h>

class PathFollower;
class PoseTracker;
class ObstacleCloud;

class AbstractLocalPlanner
{
public:
    virtual ~AbstractLocalPlanner();

    virtual void init(RobotController *controller, PoseTracker *pose_tracker,
                      const LocalPlannerParameters &opt);

    virtual void setGlobalPath(Path::Ptr path);

    virtual void setVelocity(geometry_msgs::Twist::_linear_type vector) = 0;

    virtual void setVelocity(double velocity) = 0;

    virtual void reset();

    virtual Path::Ptr updateLocalPath() = 0;

    virtual bool isNull() const;

    void setObstacleCloud(const std::shared_ptr<ObstacleCloud const> &msg);

    void addConstraint(Constraint::Ptr constraint);
    void addScorer(Scorer::Ptr scorer, double weight);

    virtual std::vector<SubPath> getAllLocalPaths() const;

protected:
    AbstractLocalPlanner();

    virtual void setParams(const LocalPlannerParameters& opt) = 0;

    Path::Ptr setPath(const std::__cxx11::string &frame_id, SubPath& local_wps, ros::Time& now);

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

    ros::Time last_update_;
};

#endif // ABSTRACT_LOCAL_PLANNER_H
