#ifndef LOCAL_PLANNER_CLASSIC_H
#define LOCAL_PLANNER_CLASSIC_H

/// PROJECT
#include <path_follower/local_planner/local_planner.h>

/// SYSTEM
#include <ros/time.h>

class LocalPlannerClassic : public LocalPlanner
{
public:
    LocalPlannerClassic(PathFollower& controller,
                            tf::Transformer &transformer,
                            const ros::Duration& update_interval);

    virtual void setGlobalPath(Path::Ptr path) override;

protected:
    template <typename NodeT>
    void getSuccessors(NodeT*& current, int& nsize, std::vector<NodeT*>& successors,
                       std::vector<NodeT>& nodes, const std::vector<Constraint::Ptr>& constraints,
                       const std::vector<bool>& fconstraints, bool repeat = false){
        successors.clear();
        double theta;
        double ori = current->orientation;
        double ox = current->x;
        double oy = current->y;
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
            const NodeT succ(x,y,theta,current,current->level_+1);
            const tf::Point succp(x,y,theta);

            if(areConstraintsSAT(succp,constraints,fconstraints)){
                int wo = -1;
                if(!isInGraph(succ,nodes,nsize,wo)){
                    nodes.at(nsize) = succ;
                    successors.push_back(&nodes.at(nsize));
                    nsize++;
                }else{
                    if(repeat){
                        successors.push_back(&nodes[wo]);
                    }
                }
            }
        }
    }

    template <typename NodeT>
    bool isInGraph(const NodeT& current, std::vector<NodeT>& nodes, int& asize, int& position){
        for(std::size_t i = 0; i < asize; ++i){
            double dis = current.distanceTo(nodes[i]);
            if(dis < 0.05){
                position = i;
                return true;
            }
        }
        return false;
    }

    bool areConstraintsSAT(const tf::Point& current, const std::vector<Constraint::Ptr>& constraints,
                           const std::vector<bool>& fconstraints);

    int transform2Odo(SubPath& waypoints, ros::Time& now);

    void initConstraintsAndScorers(const std::vector<Constraint::Ptr>& constraints,
                                   const std::vector<Scorer::Ptr>& scorer,
                                   const std::vector<bool>& fconstraints,
                                   const std::vector<double>& wscorer,
                                   SubPath& waypoints);
    void setPath(Path::Ptr& local_path, SubPath& local_wps, ros::Time& now);

    void smoothAndInterpolate(SubPath& local_wps);

    bool isNearEnough(const Waypoint& current, const Waypoint& last);

    void initIndexes();

    SubPath interpolatePath(const SubPath& path, double max_distance);
    void subdividePath(SubPath& result, Waypoint low, Waypoint up, double max_distance);
    SubPath smoothPath(const SubPath& path, double weight_data, double weight_smooth, double tolerance = 0.000001);
    std::vector<SubPath> segmentPath(const SubPath &path);
    SubPath smoothPathSegment(const SubPath& path, double weight_data, double weight_smooth, double tolerance);

protected:
    const double D_THETA = 5*M_PI/36;//Assume like the global planner 25° turn

    ros::Time last_update_;
    ros::Duration update_interval_;

    SubPath last_local_path_;
    std::size_t index1;
    std::size_t index2;

    std::vector<double> c_dist;
};

#endif // LOCAL_PLANNER_CLASSIC_H
