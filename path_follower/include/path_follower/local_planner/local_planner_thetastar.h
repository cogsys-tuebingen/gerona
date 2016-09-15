#ifndef LOCAL_PLANNER_THETASTAR_H
#define LOCAL_PLANNER_THETASTAR_H

/// PROJECT
#include <path_follower/local_planner/local_planner_star.h>

class LocalPlannerThetaStar : virtual public LocalPlannerStar
{
public:
    LocalPlannerThetaStar(PathFollower& controller, tf::Transformer &transformer,
                      const ros::Duration& update_interval);
private:
    bool tryForAlternative(LNode*& s_p);
    virtual double G(LNode*& current, std::size_t& index, std::vector<LNode*>& successors,
             const std::vector<Scorer::Ptr>& scorer, const std::vector<double>& wscorer,
             double& score) override;

    virtual void updateSucc(LNode*& current, LNode*& f_current, LNode& succ) override;
private:
    LNode alt;
};

#endif // LOCAL_PLANNER_THETASTAR_H
