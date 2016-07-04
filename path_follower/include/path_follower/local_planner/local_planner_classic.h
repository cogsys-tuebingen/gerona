#ifndef LOCAL_PLANNER_CLASSIC_H
#define LOCAL_PLANNER_CLASSIC_H

/// PROJECT
#include <path_follower/local_planner/local_planner_implemented.h>

class LocalPlannerClassic : public LocalPlannerImplemented
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
                       const std::vector<bool>& fconstraints,const std::vector<double>& wscorer
                       /*, bool repeat = false*/){
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
            NodeT succ(x,y,theta,current,current->level_+1);
            setDistances(succ,(fconstraints.at(1) || wscorer.at(4) != 0));

            if(areConstraintsSAT(succ,constraints,fconstraints)){
                int wo = -1;
                if(!isInGraph(succ,nodes,nsize,wo)){
                    nodes.at(nsize) = succ;
                    successors.push_back(&nodes.at(nsize));
                    nsize++;
                }/*else{
                    if(repeat){
                        successors.push_back(&nodes[wo]);
                    }
                }*/
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

    template <typename NodeT>
    void setDistances(NodeT& current, bool b_obst){
        double closest_dist = std::numeric_limits<double>::infinity();
        int closest_index = 0;
        if(waypoints.empty()){
            return;
        }
        for(std::size_t i = index1; i <= index2; ++i) {
            const Waypoint& wp = waypoints[i];
            double dist = std::hypot(wp.x - current.x, wp.y - current.y);
            if(dist < closest_dist) {
                closest_dist = dist;
                closest_index = i;
            }
        }
        current.d2p = closest_dist;
        current.npp = waypoints[closest_index];

        if(b_obst){
            tf::Point pt(current.x, current.y, current.orientation);
            pt = odom_to_base * pt;
            double closest_obst = std::numeric_limits<double>::infinity();
            double closest_x = std::numeric_limits<double>::infinity();
            double closest_y = std::numeric_limits<double>::infinity();
            ObstacleCloud::const_iterator point_it;
            for (point_it = obstacle_cloud_->begin(); point_it != obstacle_cloud_->end(); ++point_it){
                double x = (double)(point_it->x) - pt.x();
                double y = (double)(point_it->y) - pt.y();
                double dist = std::hypot(x, y);
                if(dist < closest_obst) {
                    closest_obst = dist;
                    closest_x = (double)(point_it->x);
                    closest_y = (double)(point_it->y);
                }
            }
            current.d2o = closest_obst;
            tf::Point tmpnop(closest_x ,closest_y,0.0);
            tmpnop = base_to_odom * tmpnop;
            current.nop = Waypoint(tmpnop.x(), tmpnop.x(), 0.0);
        }else{
            current.d2o = std::numeric_limits<double>::infinity();
        }
    }

    template <typename NodeT>
    void retrievePath(NodeT* obj, SubPath& local_wps){
        LNode* cu = obj;
        while(cu != nullptr){
            local_wps.push_back(*cu);
            cu = cu->parent_;
        }
        std::reverse(local_wps.begin(),local_wps.end());
    }

    template <typename NodeT>
    void retrieveContinuity(NodeT& wpose){
        if(last_local_path_.n()>0){
            std::size_t index = -1;
            double s_new = last_local_path_.s_new() + 0.7;
            double closest_point = std::numeric_limits<double>::infinity();
            for(std::size_t i = 0; i < last_local_path_.n(); ++i){
                double dist = std::abs(s_new - last_local_path_.s(i));
                if(dist < closest_point) {
                    closest_point = dist;
                    index = i;
                }
            }
            wpose.xp = last_local_path_.p_prim(index);
            wpose.yp = last_local_path_.q_prim(index);

            wpose.xs = last_local_path_.p_sek(index);
            wpose.ys = last_local_path_.q_sek(index);
        }
    }

    bool areConstraintsSAT(const LNode& current, const std::vector<Constraint::Ptr>& constraints,
                           const std::vector<bool>& fconstraints);

    void initScorers(const std::vector<Scorer::Ptr>& scorer,
                     const std::vector<double>& wscorer);

    void initIndexes();

    void smoothAndInterpolate(SubPath& local_wps);

    bool isNearEnough(const Waypoint& current, const Waypoint& last);

    double Score(const LNode& current, const double& dis2last,
                        const std::vector<Scorer::Ptr>& scorer, const std::vector<double>& wscorer);

    void savePath(SubPath& local_wps);

    SubPath interpolatePath(const SubPath& path, double max_distance);
    void subdividePath(SubPath& result, Waypoint low, Waypoint up, double max_distance);
    SubPath smoothPath(const SubPath& path, double weight_data, double weight_smooth, double tolerance = 0.000001);
    std::vector<SubPath> segmentPath(const SubPath &path);
    SubPath smoothPathSegment(const SubPath& path, double weight_data, double weight_smooth, double tolerance);
private:
    virtual void printNodeUsage(int& nnodes) const override;
protected:
    static constexpr double D_THETA = 5*M_PI/36;//Assume like the global planner 25° turn

    std::size_t index1;
    std::size_t index2;

    std::vector<double> c_dist;

    PathInterpolated last_local_path_;
};

#endif // LOCAL_PLANNER_CLASSIC_H
