#ifndef LOCAL_PLANNER_CLASSIC_H
#define LOCAL_PLANNER_CLASSIC_H

/// PROJECT
#include <path_follower/local_planner/local_planner.h>

class LocalPlannerClassic : public LocalPlannerImplemented
{
public:
    LocalPlannerClassic();

    virtual void setGlobalPath(Path::Ptr path) override;

    virtual void setVelocity(geometry_msgs::Twist::_linear_type vector) override;
    virtual void setVelocity(double velocity) override;

protected:
    void getSuccessors(LNode*& current, std::size_t& nsize, std::vector<LNode*>& successors,
                       std::vector<LNode>& nodes, std::vector<LNode>& twins = EMPTYTWINS,
                       bool repeat = false);

    double Heuristic(const LNode& current, const double& dis2last);

    double Cost(const LNode& current, double& score);

    double Score(const LNode& current);

    void checkQuarters(LNode child, LNode* parent, LNode& first, LNode& mid, LNode& second);

    bool createAlternative(LNode*& s_p, LNode& alt, bool allow_lines = false);

    virtual void setParams(const LocalPlannerParameters &opt) override;

private:
    void setDistances(LNode& current);

    void findClosestObstaclePoint(std::shared_ptr<ObstacleCloud const>& cloud, tf::Point& pt, double& closest_obst, double& closest_x, double& closest_y, bool& change);

    void retrieveContinuity(LNode& wpose);

    void setD2P(LNode& wpose);

    bool processPath(LNode* obj,SubPath& local_wps);

    bool isInGraph(const LNode& current, std::vector<LNode>& nodes, std::size_t& asize, int& position);

    bool areConstraintsSAT(const LNode& current);

    void initConstraints();

    void setNormalizer();

    void initIndexes(Eigen::Vector3d& pose);

    void setLastLocalPaths(std::size_t index);

    void setLastLocalPaths();

    void retrievePath(LNode* obj, SubPath& local_wps, double& l);

    void smoothAndInterpolate(SubPath& local_wps);

    SubPath interpolatePath(const SubPath& path, double max_distance);

    void subdividePath(SubPath& result, Waypoint low, Waypoint up, double max_distance);

    SubPath smoothPath(const SubPath& path, double weight_data, double weight_smooth, double tolerance = 0.000001);

    std::vector<SubPath> segmentPath(const SubPath &path);

    SubPath smoothPathSegment(const SubPath& path, double weight_data, double weight_smooth, double tolerance);

    void setStep();

    double computeFrontier(double& angle);

    virtual bool algo(Eigen::Vector3d& pose, SubPath& local_wps,
                     std::size_t& nnodes) override;

    virtual void printNodeUsage(std::size_t& nnodes) const override;

    virtual void printVelocity() override;

    virtual void printLevelReached() const override;

    virtual void initLeaves(LNode& root) = 0;

    virtual void updateLeaves(std::vector<LNode*>& successors, LNode*& current) = 0;

    virtual void updateBest(double& current_p, double& best_p, LNode*& obj, LNode*& succ) = 0;

    virtual void addLeaf(LNode*& node) = 0;

    virtual void reconfigureTree(LNode*& obj, std::vector<LNode>& nodes, double& best_p) = 0;

    virtual void setInitScores(LNode& wpose, double& dis2last) = 0;

    virtual void initQueue(LNode& root) = 0;

    virtual bool isQueueEmpty() = 0;

    virtual LNode* queueFront() = 0;

    virtual void pop(LNode*& current) = 0;

    virtual void push2Closed(LNode*& current) = 0;

    virtual void expandCurrent(LNode*& current, std::size_t& nsize, std::vector<LNode*>& successors,
                               std::vector<LNode>& nodes) = 0;

    virtual bool processSuccessor(LNode*& succ, LNode*& current,
                                  double& current_p, double& dis2last) = 0;

private:
    static constexpr double L = 0.458;//(Distance between front and rear axis of Summit XL)
    static constexpr double RL = 0.722;
    static constexpr double RW = 0.61;
    static std::vector<LNode> EMPTYTWINS;

    static std::size_t max_num_nodes_;
    static int ic_, nsucc_, max_level_;
    static double TH, length_MF, mudiv_;
    static double GL, GW, FL, beta1;
    static std::vector<double> D_THETA, RT;

    double d2p, last_s, new_s, velocity_;
    double obstacle_threshold_;
    bool fvel_, b_obst;

    std::size_t index1;
    std::size_t index2;
    int r_level,n_v;

    PathInterpolated last_local_path_;

    double step_, neig_s, FFL, beta2;
};

#endif // LOCAL_PLANNER_CLASSIC_H
