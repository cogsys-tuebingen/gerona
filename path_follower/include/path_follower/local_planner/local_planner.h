#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

/// PROJECT
#include <cslibs_utils/MathHelper.h>
#include <cslibs_utils/Stopwatch.h>
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
#include <path_follower/local_planner/curvatured_scorer.h>

class PathFollower;

class LocalPlanner
{
public:
    virtual ~LocalPlanner();

    virtual void setGlobalPath(Path::Ptr path);

    virtual void setVelocity(geometry_msgs::Twist::_linear_type vector) = 0;

    virtual void setVelocity(double velocity) = 0;

    virtual void reset();

    virtual Path::Ptr updateLocalPath(const std::vector<Constraint::Ptr>& constraints,
                                      const std::vector<Scorer::Ptr>& scorer,
                                      const std::vector<bool>& fconstraints,
                                      const std::vector<double>& wscorer,
                                      Path::Ptr& wlp) = 0;

    virtual bool isNull() const;
    virtual void setParams(int nnodes, int ic, double dis2p, double adis, double fdis, double s_angle,
                           int ia, double lmf, int max_level, double mu, double ef) = 0;

    void setObstacleCloud(const ObstacleCloud::ConstPtr &msg);


protected:
    LocalPlanner(PathFollower& controller,
                 tf::Transformer &transformer);

protected:
    PathFollower& follower_;
    tf::Transformer &transformer_;

    PathInterpolated global_path_;

    tf::StampedTransform initial_map_to_odom_, base_to_odom, lastbase_to_odom;

    tf::Transform odom_to_base, odom_to_lastbase;

    ObstacleCloud::ConstPtr obstacle_cloud_, last_obstacle_cloud_;
};

#endif // LOCAL_PLANNER_H
