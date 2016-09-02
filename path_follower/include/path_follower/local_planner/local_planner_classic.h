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

    virtual void setVelocity(geometry_msgs::Twist::_linear_type vector) override;
    virtual void setVelocity(double velocity) override;
    virtual void setParams(int nnodes, int ic, double dis2p, double dis2o, double s_angle) override;

protected:
    template <typename NodeT>
    void getSuccessors(NodeT*& current, int& nsize, std::vector<NodeT*>& successors,
                       std::vector<NodeT>& nodes, const std::vector<Constraint::Ptr>& constraints,
                       const std::vector<bool>& fconstraints,const std::vector<double>& wscorer,
                       std::vector<NodeT>& twins = EMPTYTWINS, bool repeat = false){
        successors.clear();
        twins.resize(3);
        bool add_n = true;
        double ori = current->orientation;
        double trax = L*std::cos(ori)/2.0;
        double tray = L*std::sin(ori)/2.0;
        double ox = current->x - trax;
        double oy = current->y - tray;
        for(int i = 0; i < 3; ++i){
            double x,y,theta,rt;
            if(i == 0){// straight
                theta = ori;
                x = ox + step_*std::cos(theta) + trax;
                y = oy + step_*std::sin(theta) + tray;
                rt = std::numeric_limits<double>::infinity();
            }else{
                switch (i) {
                case 1:// right
                    rt = -RT;
                    theta = -D_THETA;
                    break;
                case 2:// left
                    rt = RT;
                    theta = D_THETA;
                    break;
                default:
                    break;
                }
                theta = MathHelper::AngleClamp(ori + theta);
                trax = L*std::cos(theta)/2.0;
                tray = L*std::sin(theta)/2.0;
                x = ox + rt*(std::sin(theta)-std::sin(ori)) + trax;
                y = oy + rt*(-std::cos(theta)+std::cos(ori)) + tray;
                
            }
            NodeT succ(x,y,theta,current,rt,current->level_+1);
            setDistances(succ,(fconstraints.back() || wscorer.back() != 0));

            if(areConstraintsSAT(succ,constraints,fconstraints)){
                int wo = -1;
                if(!isInGraph(succ,nodes,nsize,wo)){
                    if(add_n){
                        nodes.at(nsize) = succ;
                        successors.push_back(&nodes.at(nsize));
                        nsize++;
                        if(nsize >= nnodes_){
                            add_n = false;
                        }
                    }
                }else{
                    if(repeat){
                        twins.at(i) = succ;
                        nodes[wo].twin_ = &twins.at(i);
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
            if(dis < neig_s){
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
        for(std::size_t i = index1; i <= index2; ++i) {
            const Waypoint& wp = waypoints[i];
            double dist = std::hypot(wp.x - current.x, wp.y - current.y);
            if(dist < closest_dist) {
                closest_dist = dist;
                closest_index = i;
            }
        }

        double dis = 0.0;
        if(closest_index == index1){
            while(closest_index != 0){
                const int c_i = closest_index - 1;
                const Waypoint& wp = waypoints[c_i];
                double dist = std::hypot(wp.x - current.x, wp.y - current.y);
                if(dist < closest_dist) {
                    closest_dist = dist;
                    closest_index = c_i;
                }else{
                    break;
                }
            }
            if(closest_index == 0){
                const Waypoint& p0 = waypoints[0];
                const Waypoint& p1 = waypoints[1];
                double x = p1.x - p0.x;
                double y = p1.y - p0.y;
                double a_next = std::atan2(y,x);
                x = current.x - p0.x;
                y = current.y - p0.y;
                double a_point = std::atan2(y,x);
                double adiff = std::abs(MathHelper::AngleClamp(a_next - a_point));
                if(adiff > M_PI_2){
                    double h = std::hypot(p0.x - current.x, p0.y - current.y);
                    dis = h*std::cos(adiff);
                    closest_dist = h*std::sin(adiff);
                }
            }
        }
        if(closest_index == index2){
            std::size_t last_p = waypoints.size() - 1;
            while(closest_index != last_p){
                const int c_i = closest_index + 1;
                const Waypoint& wp = waypoints[c_i];
                double dist = std::hypot(wp.x - current.x, wp.y - current.y);
                if(dist < closest_dist) {
                    closest_dist = dist;
                    closest_index = c_i;
                }else{
                    break;
                }
            }
        }
        current.d2p = closest_dist;
        current.npp = waypoints[closest_index];
        current.s = current.npp.s + dis;

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
    void retrievePath(NodeT* obj, SubPath& local_wps, double& l){
        LNode* cu = obj;
        r_level = cu->level_;
        l = 0.0;
        while(cu != nullptr){
            local_wps.push_back(*cu);
            if(local_wps.size() != 1){
                l += local_wps.back().distanceTo(local_wps.at(local_wps.size()-2));
            }
            if(cu->parent_ != nullptr){
                if(cu->radius_ != std::numeric_limits<double>::infinity()){
                    const LNode* parent = cu->parent_;
                    double theta = MathHelper::AngleClamp(cu->orientation - parent->orientation);
                    const double rt = cu->radius_;
                    const double step = theta/((double)(ic_ + 1));
                    double ori = parent->orientation;
                    double trax = L*std::cos(ori)/2.0;
                    double tray = L*std::sin(ori)/2.0;
                    double ox = parent->x - trax;
                    double oy = parent->y - tray;
                    for(int i = ic_; i >= 1; --i){
                        theta = MathHelper::AngleClamp(ori + ((double)i)*step);
                        trax = L*std::cos(theta)/2.0;
                        tray = L*std::sin(theta)/2.0;
                        double x = ox + rt*(std::sin(theta)-std::sin(ori)) + trax;
                        double y = oy + rt*(-std::cos(theta)+std::cos(ori)) + tray;
                        Waypoint bc(x,y,theta);
                        bc.s = parent->s + ((double)i)*step*rt;
                        local_wps.push_back(bc);
                        l += local_wps.back().distanceTo(local_wps.at(local_wps.size()-2));
                    }
                }
            }
            cu = cu->parent_;
        }
        std::reverse(local_wps.begin(),local_wps.end());
    }

    template <typename NodeT>
    void retrieveContinuity(NodeT& wpose){
        global_path_.set_s_new(new_s);
        if(last_local_path_.n()>0){
            std::size_t index = -1;
            double closest_point = std::numeric_limits<double>::infinity();
            for(std::size_t i = 0; i < last_local_path_.n(); ++i){
                double x = last_local_path_.p(i) - wpose.x;
                double y = last_local_path_.q(i) - wpose.y;
                double dist = std::hypot(x, y);
                if(dist < closest_point) {
                    closest_point = dist;
                    index = i;
                }
            }            
            double curv = last_local_path_.curvature(index);

            if(curv == 0.0){
                wpose.radius_ = std::numeric_limits<double>::infinity();
            }else{
                wpose.radius_ = 1.0/curv;
            }

            setLLP(index + 1);
        }
    }

    template <typename NodeT>
    void setD2P(NodeT& wpose){
        double px = wpose.x - wpose.npp.x;
        double py = wpose.y - wpose.npp.y;

        double x = px + step_*std::cos(wpose.orientation);
        double y = py + step_*std::sin(wpose.orientation);
        double d1 = std::hypot(x, y);

        x = px + stepc_*std::cos(MathHelper::AngleClamp(wpose.orientation + D_THETA/2.0));
        y = py + stepc_*std::sin(MathHelper::AngleClamp(wpose.orientation + D_THETA/2.0));
        double d2 = std::hypot(x, y);

        x = px + stepc_*std::cos(MathHelper::AngleClamp(wpose.orientation - D_THETA/2.0));
        y = py + stepc_*std::sin(MathHelper::AngleClamp(wpose.orientation - D_THETA/2.0));
        double d3 = std::hypot(x, y);

        d2p = max(max(wpose.d2p,d1),max(d2,d3));
    }

    template <typename NodeT>
    bool processPath(NodeT* obj,SubPath& local_wps){
        double length;
        retrievePath(obj, local_wps,length);
        if(length < 0.12){
            return false;
        }
        last_s = global_path_.s_new();
        std::size_t i_new = local_wps.size() - 1;
        double dis = velocity_ * update_interval_.toSec();
        for(std::size_t i = 1; i < local_wps.size(); ++i){
            if(local_wps.at(i).s - local_wps.at(0).s >= dis){
                i_new = i;
                break;
            }
        }
        global_path_.set_s_new(local_wps.at(i_new).s);
        smoothAndInterpolate(local_wps);
        if(tooClose){
            wlp_.insert(wlp_.end(),local_wps.begin(),local_wps.end());
        }
        last_local_path_.interpolatePath(local_wps, "/odom");
        return true;
    }

    bool areConstraintsSAT(const LNode& current, const std::vector<Constraint::Ptr>& constraints,
                           const std::vector<bool>& fconstraints);

    void initConstraints(const std::vector<Constraint::Ptr>& constraints,
                                              const std::vector<bool>& fconstraints);

    void initIndexes(Eigen::Vector3d& pose);

    void smoothAndInterpolate(SubPath& local_wps);

    double Heuristic(const LNode& current, const double& dis2last);

    double Cost(const LNode& current, const std::vector<Scorer::Ptr>& scorer,
                const std::vector<double>& wscorer, double& score);

    double Score(const LNode& current, const std::vector<Scorer::Ptr>& scorer,
                 const std::vector<double>& wscorer);

    void setStep();

    void setLLP(std::size_t index);

    void setLLP();

    SubPath interpolatePath(const SubPath& path, double max_distance);
    void subdividePath(SubPath& result, Waypoint low, Waypoint up, double max_distance);
    SubPath smoothPath(const SubPath& path, double weight_data, double weight_smooth, double tolerance = 0.000001);
    std::vector<SubPath> segmentPath(const SubPath &path);
    SubPath smoothPathSegment(const SubPath& path, double weight_data, double weight_smooth, double tolerance);
private:
    virtual void printNodeUsage(int& nnodes) const override;
    virtual void printVelocity() override;
    virtual void printLevelReached() const override;
protected:
    static constexpr double L = 0.46;
    static std::vector<LNode> EMPTYTWINS;

    static int nnodes_,ic_;
    static double D_THETA, RT;

    double d2p, last_s, new_s, velocity_;
    bool fvel_;

    std::size_t index1;
    std::size_t index2;
    int r_level;

    PathInterpolated last_local_path_;

    double step_,stepc_,neig_s;
};

#endif // LOCAL_PLANNER_CLASSIC_H
