/// HEADER
#include <path_follower/local_planner/local_planner_thetastar.h>

/// PROJECT
#include <path_follower/pathfollower.h>

LocalPlannerThetaStar::LocalPlannerThetaStar(PathFollower &controller, tf::Transformer &transformer,
                                             const ros::Duration &update_interval)
    :LocalPlannerStar(controller,transformer,update_interval),alt()
{

}

double LocalPlannerThetaStar::G(HNode*& current, std::size_t& index, std::vector<HNode*>& successors,
                            const std::vector<Scorer::Ptr>& scorer, const std::vector<double>& wscorer,
                            double& score){
    double tentative_gScore = current->gScore_ ;
    LNode* succ;
    if(successors[index]->twin_ != nullptr){
        succ = successors[index]->twin_;
    }else{
        succ = successors[index];
    }
    tentative_gScore += Cost(*(succ), scorer, wscorer, score);
    if(tryForAlternative(succ)){
        double score1;
        //double tentative_gScore1 = (HNode*)(current->parent_)->gScore;
        ROS_INFO_STREAM("succ = (" << succ->x << "," << succ->y << "," << succ->orientation << ")");
        ROS_INFO_STREAM("alt = (" << alt.x << "," << alt.y << "," << alt.orientation << ")");
    }
    return tentative_gScore;
}

bool LocalPlannerThetaStar::tryForAlternative(LNode*& s_p){
    LNode* s = s_p->parent_;
    if(s->parent_ == nullptr){
        return false;
    }
    LNode* parent = s->parent_;
    double x = s->x - parent->x;
    double y = s->y - parent->y;
    double d = std::hypot(x,y);
    double theta_b = std::atan2(y,x);
    x = s_p->x - s->x;
    y = s_p->y - s->y;
    double c = std::hypot(x,y);
    double theta_r = std::atan2(y,x);
    double gamma  = MathHelper::AngleClamp(theta_r - theta_b);
    double theta_p = MathHelper::AngleClamp(theta_b - parent->orientation);
    double divisor = c*std::sin(MathHelper::AngleClamp(gamma + theta_p)) + d*std::sin(theta_p);
    if(std::abs(divisor) <= std::numeric_limits<double>::epsilon()){
        return false;
    }
    divisor *= 2.0;
    x = s_p->x - parent->x;
    y = s_p->y - parent->y;
    double a = std::hypot(x,y);
    double R = (a*a)/divisor;
    double psi_v = atan2(L,std::abs(R));
    if (psi_v > TH){
        return false;
    }
    double theta_n = (R >= 0.0?1.0:-1.0)*std::acos(1-(0.5*a*a)/(R*R));
    alt = *s_p;
    alt.orientation = MathHelper::AngleClamp(parent->orientation + theta_n);
    alt.parent_ = parent;
    alt.radius_ = R;
    return true;
}
