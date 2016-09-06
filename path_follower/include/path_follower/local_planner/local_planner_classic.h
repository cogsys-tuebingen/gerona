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
    void getSuccessors(LNode*& current, int& nsize, std::vector<LNode*>& successors,
                       std::vector<LNode>& nodes, const std::vector<Constraint::Ptr>& constraints,
                       const std::vector<bool>& fconstraints,const std::vector<double>& wscorer,
                       std::vector<LNode>& twins = EMPTYTWINS, bool repeat = false);

    bool isInGraph(const LNode& current, std::vector<LNode>& nodes, int& asize, int& position);

    void setDistances(LNode& current, bool b_obst);

    void retrievePath(LNode* obj, SubPath& local_wps, double& l);

    void retrieveContinuity(LNode& wpose);

    void setD2P(LNode& wpose);

    bool processPath(LNode* obj,SubPath& local_wps);

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
    static double D_THETA, RT, TH;

    double d2p, last_s, new_s, velocity_;
    bool fvel_;

    std::size_t index1;
    std::size_t index2;
    int r_level,n_v;

    PathInterpolated last_local_path_;

    double step_,stepc_,neig_s;
};

#endif // LOCAL_PLANNER_CLASSIC_H
