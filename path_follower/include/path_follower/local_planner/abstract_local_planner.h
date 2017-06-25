#ifndef ABSTRACT_LOCAL_PLANNER_H
#define ABSTRACT_LOCAL_PLANNER_H

/// PROJECT
#include <cslibs_utils/MathHelper.h>
#include <cslibs_utils/Stopwatch.h>
#include <path_follower/utils/path.h>
#include <path_follower/utils/path_interpolated.h>
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

class PathFollowerParameters;
class LocalPlannerParameters;

class AbstractLocalPlanner
{
public:
    virtual ~AbstractLocalPlanner();

    virtual void init(RobotController *controller, PoseTracker *pose_tracker);

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

    static void smoothAndInterpolate(SubPath& local_wps);
    static SubPath interpolatePath(const SubPath& path, double max_distance);
    static void subdividePath(SubPath& result, Waypoint low, Waypoint up, double max_distance);
    static SubPath smoothPath(const SubPath& path, double weight_data, double weight_smooth, double tolerance = 0.000001);
    static SubPath smoothPathSegment(const SubPath& path, double weight_data, double weight_smooth, double tolerance);


protected:
    AbstractLocalPlanner();

    virtual void setParams(const LocalPlannerParameters& opt) = 0;

    void setPath(const Path::Ptr &local_wps, const ros::Time& now);
    Path::Ptr setPath(const std::string &frame_id, const SubPath& local_wps, const ros::Time& now);

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
