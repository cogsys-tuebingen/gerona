#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

/// PROJECT
#include <utils_general/MathHelper.h>
#include <utils_general/Stopwatch.h>
#include <path_follower/utils/path.h>
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
                                      const std::vector<Scorer::Ptr>& scorer) = 0;

    virtual bool isNull() const;

protected:
    LocalPlanner(PathFollower& controller,
                 tf::Transformer &transformer);

    template <typename NodeT>
    void getSuccessors(const NodeT& current, int index, std::vector<int>& successors,
                       std::vector<NodeT>& nodes, const std::vector<Constraint::Ptr>& constraints,
                       bool repeat = false){
        successors.clear();
        double theta;
        double ori = current.orientation;
        double ox = current.x;
        double oy = current.y;
        for(int i = 0; i < 3; ++i){
            switch (i) {
            case 0:// straight
                theta = ori;
                break;
            case 1:// right
                theta = ori - D_THETA;
                break;
            case 2:// left
                theta = ori + D_THETA;
                break;
            default:
                break;
            }

            double x = ox + 0.15*std::cos(theta);
            double y = oy + 0.15*std::sin(theta);
            const NodeT succ(x,y,theta,index,current.level_+1);
            const tf::Point succp(x,y,theta);

            if(constraints.at(0)->isSatisfied(succp) && constraints.at(2)->isSatisfied(succp)){
                int wo = -1;
                if(!isInGraph(succ,nodes,wo)){
                    successors.push_back(nodes.size());
                    nodes.push_back(succ);
                }else{
                    if(repeat){
                        successors.push_back(wo);
                    }
                }
            }
        }
    }
    bool isNearEnough(const Waypoint& current, const Waypoint& last);
    bool isInGraph(const Waypoint& current, std::vector<LNode>& nodes, int& position);

    SubPath interpolatePath(const SubPath& path, double max_distance);
    void subdividePath(SubPath& result, Waypoint low, Waypoint up, double max_distance);
    SubPath smoothPath(const SubPath& path, double weight_data, double weight_smooth, double tolerance = 0.000001);
    std::vector<SubPath> segmentPath(const SubPath &path);
    SubPath smoothPathSegment(const SubPath& path, double weight_data, double weight_smooth, double tolerance);

protected:
    const double D_THETA = 5*M_PI/36;//Assume like the global planner 25° turn

    PathFollower& follower_;
    tf::Transformer &transformer_;

    Path::Ptr global_path_;
    SubPath last_local_path_;

    tf::StampedTransform initial_map_to_odom_;
};

#endif // LOCAL_PLANNER_H
