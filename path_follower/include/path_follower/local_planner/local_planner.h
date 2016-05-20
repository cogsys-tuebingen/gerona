#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

/// PROJECT
#include <utils_general/MathHelper.h>
#include <utils_general/Stopwatch.h>
#include <path_follower/utils/path.h>
#include <path_follower/local_planner/constraint.h>
#include <path_follower/local_planner/dis2path_constraint.h>
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
                                      const std::vector<Scorer::Ptr>& scorer) = 0;

    virtual bool isNull() const;

protected:
    LocalPlanner(PathFollower& controller,
                 tf::Transformer &transformer);

    void getSuccessors(const Waypoint& current, int index, std::vector<int>& successors,
                       std::vector<Waypoint>& nodes, std::vector<int>& parents,
                       std::vector<int>& level, const std::vector<Constraint::Ptr>& constraints);
    bool isNearEnough(const Waypoint& current, const Waypoint& last);
    bool isInGraph(const Waypoint& current, std::vector<Waypoint>& nodes);

    std::vector<Waypoint> interpolatePath(const std::vector<Waypoint>& path, double max_distance);
    void subdividePath(std::vector<Waypoint>& result, Waypoint low, Waypoint up, double max_distance);
    std::vector<Waypoint> smoothPath(const std::vector<Waypoint>& path, double weight_data, double weight_smooth, double tolerance = 0.000001);
    std::vector<std::vector<Waypoint>> segmentPath(const std::vector<Waypoint> &path);
    std::vector<Waypoint> smoothPathSegment(const std::vector<Waypoint>& path, double weight_data, double weight_smooth, double tolerance);

protected:
    const double D_THETA = 5*M_PI/36;//Assume like the global planner 25° turn

    PathFollower& follower_;
    tf::Transformer &transformer_;

    Path::Ptr global_path_;

    tf::StampedTransform initial_map_to_odom_;
};

#endif // LOCAL_PLANNER_H
