/// HEADER
#include <path_follower/local_planner/local_planner_classic.h>

/// PROJECT
#include <path_follower/pathfollower.h>

int LocalPlannerClassic::nnodes_ = 300;
int LocalPlannerClassic::ic_ = 3;
double LocalPlannerClassic::RT = std::numeric_limits<double>::infinity();
double LocalPlannerClassic::D_THETA = 0.0;

LocalPlannerClassic::LocalPlannerClassic(PathFollower &follower,
                                 tf::Transformer& transformer,
                                 const ros::Duration& update_interval)
    : LocalPlannerImplemented(follower, transformer, update_interval),
      d2p(0.0),last_s(0.0), new_s(0.0),velocity_(0.0),fvel_(false),index1(-1), index2(-1),
      step_(0.0),stepc_(0.0),neig_s(0.0)
{

}

void LocalPlannerClassic::setGlobalPath(Path::Ptr path){
    LocalPlannerImplemented::setGlobalPath(path);
    last_s = 0.0;
    new_s = 0.0;
}

void LocalPlannerClassic::setVelocity(geometry_msgs::Twist::_linear_type vector){
    if(!fvel_){
        double tmpv = sqrt(vector.x*vector.x + vector.y*vector.y + vector.z*vector.z);
        if(tmpv > 0.05){
            velocity_ = tmpv;
        }
    }
    setStep();
}

void LocalPlannerClassic::setVelocity(double velocity){
    velocity_ = velocity;
    fvel_ = true;
    setStep();
}

void LocalPlannerClassic::setStep(){
    double dis = velocity_ * update_interval_.toSec();
    step_ = 3.0*dis/10.0;
    D_THETA = MathHelper::AngleClamp(step_/RT);
    double H_D_THETA = D_THETA/2.0;
    stepc_ = 2.0*RT*std::sin(H_D_THETA);
    neig_s = stepc_*sin(H_D_THETA);
    Dis2Path_Constraint::setAngle(H_D_THETA);
}

//borrowed from path_planner/planner_node.cpp
SubPath LocalPlannerClassic::interpolatePath(const SubPath& path, double max_distance){
    unsigned n = path.size();
    std::vector<Waypoint> result;
    if(n < 2) {
        return result;
    }

    result.push_back(path[0]);

    for(unsigned i = 1; i < n; ++i){
        const Waypoint* current = &path[i];
        // split the segment, iff it is to large
        subdividePath(result, result.back(), *current, max_distance);
        // add the end of the segment (is not done, when splitting)
        result.push_back(*current);
    }
    return result;
}

//borrowed from path_planner/planner_node.cpp
void LocalPlannerClassic::subdividePath(SubPath& result, Waypoint low, Waypoint up, double max_distance){
    double distance = low.distanceTo(up);
    if(distance > max_distance) {
        // split half way between the lower and the upper node
        Waypoint halfway(low.x,low.y,low.orientation);

        halfway.x += (up.x - low.x) / 2.0;
        halfway.y += (up.y - low.y) / 2.0;

        halfway.orientation = (up.orientation + low.orientation) / 2.0;
        // first recursive descent in lower part
        subdividePath(result, low, halfway, max_distance);
        // then add the half way point
        result.push_back(halfway);
        // then descent in upper part
        subdividePath(result, halfway, up, max_distance);
    }
}

//borrowed from path_planner/planner_node.cpp
SubPath LocalPlannerClassic::smoothPath(const SubPath& path, double weight_data, double weight_smooth, double tolerance){
    SubPath result;
    int n = path.size();
    if(n < 2) {
        return result;
    }
    // find segments
    std::vector<SubPath> segments = segmentPath(path);
    // smooth segments and merge results
    for(const SubPath& segment : segments) {
        SubPath smoothed_segment = smoothPathSegment(segment, weight_data, weight_smooth, tolerance);
        result.insert(result.end(), smoothed_segment.begin(), smoothed_segment.end());
    }

    return result;
}

//borrowed from path_planner/planner_node.cpp
std::vector<SubPath> LocalPlannerClassic::segmentPath(const std::vector<Waypoint> &path){
                                   std::vector<SubPath> result;

                                   int n = path.size();
                                   if(n < 2) {
                                       return result;
                                   }

                                   SubPath current_segment;

                                   const Waypoint * last_point = &path[0];
                                   current_segment.push_back(*last_point);

                                   for(int i = 1; i < n; ++i){
                                       const Waypoint* current_point = &path[i];

                                       // append to current segment
                                       current_segment.push_back(*current_point);

                                       bool is_the_last_node = i == n-1;
                                       bool segment_ends_with_this_node = false;

                                       if(is_the_last_node) {
                                           // this is the last node
                                           segment_ends_with_this_node = true;

                                       } else {
                                           const Waypoint* next_point = &path[i+1];

                                           // if angle between last direction and next direction to large -> segment ends
                                           double diff_last_x = current_point->x - last_point->x;
                                           double diff_last_y = current_point->y - last_point->y;
                                           double last_angle = std::atan2(diff_last_y, diff_last_x);

                                           double diff_next_x = next_point->x - current_point->x;
                                           double diff_next_y = next_point->y - current_point->y;
                                           double next_angle = std::atan2(diff_next_y, diff_next_x);

                                           if(std::abs(MathHelper::AngleClamp(last_angle - next_angle)) > M_PI / 2.0) {
                                               // new segment!
                                               // current node is the last one of the old segment
                                               segment_ends_with_this_node = true;
                                           }
                                       }

                                       if(segment_ends_with_this_node) {
                                           result.push_back(current_segment);

                                           current_segment.clear();

                                           if(!is_the_last_node) {
                                               // begin new segment

                                               // current node is also the first one of the new segment
                                               current_segment.push_back(*current_point);
                                           }
                                       }

                                       last_point = current_point;
                                   }

                                   return result;
}

//borrowed from path_planner/planner_node.cpp
SubPath LocalPlannerClassic::smoothPathSegment(const SubPath& path, double weight_data, double weight_smooth, double tolerance){
    SubPath new_path(path);

    unsigned n = path.size();
    if(n < 2) {
        return new_path;
    }

    double last_change = -2 * tolerance;
    double change = 0;

    int offset = 2;

    while(change > last_change + tolerance) {
        last_change = change;
        change = 0;
        Waypoint origin(0,0,0);

        for(unsigned i = offset; i < n-offset; ++i){
            Waypoint path_i = path[i];
            Waypoint new_path_i = new_path[i];
            Waypoint new_path_ip1 = new_path[i+1];
            Waypoint new_path_im1 = new_path[i-1];

            Waypoint deltaData(weight_data*(path_i.x - new_path_i.x),
                               weight_data*(path_i.y - new_path_i.y),0.0);

            new_path_i.x = new_path_i.x + deltaData.x;
            new_path_i.y = new_path_i.y + deltaData.y;

            Waypoint deltaSmooth(weight_smooth * (new_path_ip1.x + new_path_im1.x - 2* new_path_i.x),
                                 weight_smooth * (new_path_ip1.y + new_path_im1.y - 2* new_path_i.y),
                                 0.0);
            new_path_i.x = new_path_i.x + deltaSmooth.x;
            new_path_i.y = new_path_i.y + deltaSmooth.y;

            new_path[i].x = new_path_i.x;
            new_path[i].y = new_path_i.y;


            change += deltaData.distanceTo(origin) + deltaSmooth.distanceTo(origin);
        }
    }

    // update orientations
    double a = new_path[0].orientation;
    Waypoint current = new_path[0];
    Waypoint next = new_path[1];

    double dx = next.x - current.x;
    double dy = next.y - current.y;

    Eigen::Vector2d looking_dir_normalized(std::cos(a), std::sin(a));
    Eigen::Vector2d delta(dx, dy);
    const double theta_diff = std::acos(delta.dot(looking_dir_normalized) / delta.norm());

    // decide whether to drive forward or backward
    bool is_backward = (theta_diff > M_PI_2 || theta_diff < -M_PI_2) ;

    for(unsigned i = 1; i < n-1; ++i){
        Waypoint next = new_path[i+1];
        Waypoint prev = new_path[i-1];

        Waypoint delta(next.x - prev.x, next.y - prev.y, 0.0);
        double angle = std::atan2(delta.y, delta.x);

        if(is_backward) {
            angle = MathHelper::AngleClamp(angle + M_PI);
        }

        new_path[i].orientation = angle;
    }

    return new_path;
}

void LocalPlannerClassic::initIndexes(Eigen::Vector3d& pose){
    std::size_t j = 0;
    index1 = j;
    double c_s = global_path_.s(j);
    double closest_dist = std::numeric_limits<double>::infinity();
    while(c_s <= global_path_.s_new()){
        if(c_s >= last_s){
            double x = global_path_.p(j) - pose(0);
            double y = global_path_.q(j) - pose(1);
            double dist = std::hypot(x, y);
            if(dist < closest_dist) {
                closest_dist = dist;
                index1 = j;
            }
        }
        ++j;
        if(j >= global_path_.n()){
            break;
        }
        c_s = global_path_.s(j);
    }
    double s_new = global_path_.s(index1) + 3.0 * velocity_ * update_interval_.toSec();
    closest_dist = std::numeric_limits<double>::infinity();
    for(std::size_t i = index1; i < global_path_.n(); ++i){
        double dist = std::abs(s_new - global_path_.s(i));
        if(dist < closest_dist) {
            closest_dist = dist;
            index2 = i;
        }
    }

    new_s = global_path_.s(index1);
}

void LocalPlannerClassic::initConstraints(const std::vector<Constraint::Ptr>& constraints,
                                          const std::vector<bool>& fconstraints){
    if(fconstraints.at(0)){
        std::dynamic_pointer_cast<Dis2Path_Constraint>(constraints.at(0))->setParams(d2p, stepc_);
    }
}

bool LocalPlannerClassic::areConstraintsSAT(const LNode& current, const std::vector<Constraint::Ptr>& constraints,
                       const std::vector<bool>& fconstraints){
    bool rval = true;
    for(std::size_t i = 0; i < constraints.size(); ++i){
        if(fconstraints.at(i)){
            rval = rval && constraints.at(i)->isSatisfied(current);
        }
    }
    return rval;
}

void LocalPlannerClassic::smoothAndInterpolate(SubPath& local_wps){
    //interpolate
    local_wps = interpolatePath(local_wps, 0.5);
    //smoothing
    local_wps = smoothPath(local_wps, 0.6, 0.15);
    //final interpolate
    local_wps = interpolatePath(local_wps, 0.1);
    //final smoothing
    local_wps = smoothPath(local_wps, 2.0, 0.4);
}

void LocalPlannerClassic::printNodeUsage(int& nnodes) const{
    ROS_INFO_STREAM("# Nodes: " << nnodes);
}

double LocalPlannerClassic::Score(const LNode& current, const double& dis2last,
                        const std::vector<Scorer::Ptr>& scorer, const std::vector<double>& wscorer){
    double score = dis2last - current.s;
    for(std::size_t i = 0; i < scorer.size(); ++i){
        score += ((wscorer.at(i) != 0.0)?(wscorer.at(i)*scorer.at(i)->score(current)):0.0);
    }
    return score;
}

void LocalPlannerClassic::setLLP(std::size_t index){
    SubPath tmp_p = (SubPath)last_local_path_;
    wlp_.clear();
    wlp_.assign(tmp_p.begin(),tmp_p.begin() + index);
}

void LocalPlannerClassic::setLLP(){
    setLLP(last_local_path_.n());
}

void LocalPlannerClassic::setParams(int nnodes, int ic, double dis2p, double dis2o, double s_angle){
    nnodes_ = nnodes;
    ic_ = ic;
    double th = s_angle*M_PI/180.0;
    RT = L/std::tan(th);
    Dis2Path_Constraint::setLimits(dis2p,dis2o);
    Dis2Obst_Constraint::setLimit(dis2o);
}

void LocalPlannerClassic::printVelocity(){
    ROS_INFO_STREAM("v = " << velocity_ << " m/s");
    if(fvel_){
        fvel_ = false;
    }
}

