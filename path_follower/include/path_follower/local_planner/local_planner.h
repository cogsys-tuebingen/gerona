#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

/// PROJECT
#include <cslibs_utils/MathHelper.h>
#include <cslibs_utils/Stopwatch.h>
#include <path_follower/utils/path.h>
#include <path_follower/utils/path_interpolated.h>
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

    virtual void init(RobotController *controller, PoseTracker *pose_tracker, const ros::Duration& update_interval);

    virtual void setGlobalPath(Path::Ptr path);

    virtual void setVelocity(geometry_msgs::Twist::_linear_type vector) = 0;

    virtual void setVelocity(double velocity) = 0;

    virtual void reset();

    virtual Path::Ptr updateLocalPath(Path::Ptr& wlp) = 0;

    virtual bool isNull() const;
    virtual void setParams(int nnodes, int ic, double dis2p, double adis, double fdis, double s_angle,
                           int ia, double lmf, int max_level, double mu, double ef) = 0;

    void setObstacleCloud(const std::shared_ptr<ObstacleCloud const> &msg);

    void addConstraint(Constraint::Ptr constraint);
    void addScorer(Scorer::Ptr scorer, double weight);

protected:
    LocalPlanner();


protected:
    RobotController* controller_;
    PoseTracker* pose_tracker_;
    tf::Transformer* transformer_;

    std::vector<Constraint::Ptr> constraints;
    std::vector<Scorer::Ptr> scorers;

    PathInterpolated global_path_;

    ros::Duration update_interval_;

    std::shared_ptr<ObstacleCloud const> obstacle_cloud_, last_obstacle_cloud_;
};

#endif // LOCAL_PLANNER_H
