#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

/// PROJECT
#include <utils_general/MathHelper.h>
#include <utils_general/Stopwatch.h>
#include <path_follower/utils/path.h>
#include <path_follower/utils/path_interpolated.h>
#include <path_follower/utils/obstaclecloud.hpp>
#include <path_follower/local_planner/constraint.h>
#include <path_follower/local_planner/dis2path_constraint.h>
#include <path_follower/local_planner/dis2obst_constraint.h>
#include <path_follower/local_planner/scorer.h>
#include <path_follower/local_planner/dis2pathd_scorer.h>
#include <path_follower/local_planner/dis2pathp_scorer.h>
#include <path_follower/local_planner/dis2obst_scorer.h>
#include <path_follower/local_planner/level_scorer.h>
#include <path_follower/local_planner/curvature_scorer.h>

class PathFollower;

class LocalPlanner
{
public:
    virtual ~LocalPlanner();

    virtual void setGlobalPath(Path::Ptr path);

    virtual void setVelocity(geometry_msgs::Twist::_linear_type vector) = 0;

    virtual void setVelocity(double velocity) = 0;

    virtual Path::Ptr updateLocalPath(const std::vector<Constraint::Ptr>& constraints,
                                      const std::vector<Scorer::Ptr>& scorer,
                                      const std::vector<bool>& fconstraints,
                                      const std::vector<double>& wscorer,
                                      Path::Ptr& wlp) = 0;

    virtual bool isNull() const;
    virtual void setParams(int nnodes, int ic, double dis2p, double dis2o, double s_angle, int ia) = 0;

    void setObstacleCloud(const ObstacleCloud::ConstPtr &msg);


protected:
    LocalPlanner(PathFollower& controller,
                 tf::Transformer &transformer);

protected:
    PathFollower& follower_;
    tf::Transformer &transformer_;

    PathInterpolated global_path_;

    tf::StampedTransform initial_map_to_odom_, base_to_odom;

    tf::Transform odom_to_base;

    ObstacleCloud::ConstPtr obstacle_cloud_;
};

#endif // LOCAL_PLANNER_H
