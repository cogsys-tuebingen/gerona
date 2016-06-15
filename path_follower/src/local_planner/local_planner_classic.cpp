/// HEADER
#include <path_follower/local_planner/local_planner_classic.h>

/// PROJECT
#include <path_follower/pathfollower.h>

LocalPlannerClassic::LocalPlannerClassic(PathFollower &follower,
                                 tf::Transformer& transformer,
                                 const ros::Duration& update_interval)
    : LocalPlanner(follower, transformer), last_update_(0), update_interval_(update_interval),
      last_local_path_(), index1(-1), index2(-1), c_dist()
{

}

void LocalPlannerClassic::setGlobalPath(Path::Ptr path)
{
    LocalPlanner::setGlobalPath(path);
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

                                   for(int i = 0; i < n; ++i){
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

bool LocalPlannerClassic::isNearEnough(const Waypoint& current, const Waypoint& last){
    if(current.distanceTo(last) <= 0.05){
        return true;
    }
    return false;
}

void LocalPlannerClassic::initIndexes(){
    double s_new = global_path_.s_new() + 1.5;
    double closest_dist1 = std::numeric_limits<double>::infinity();
    double closest_dist2 = std::numeric_limits<double>::infinity();
    for(std::size_t i = 0; i < global_path_.n(); ++i){
        double dist1 = std::abs(global_path_.s_new() - global_path_.s(i));
        double dist2 = std::abs(s_new - global_path_.s(i));
        if(dist1 < closest_dist1) {
            closest_dist1 = dist1;
            index1 = i;
        }
        if(dist2 < closest_dist2) {
            closest_dist2 = dist2;
            index2 = i;
        }
    }

    c_dist.clear();
    for(std::size_t i = index1; i <= index2; ++i){
        c_dist.push_back(global_path_.s(i));
    }
}

bool LocalPlannerClassic::areConstraintsSAT(const tf::Point& current, const std::vector<Constraint::Ptr>& constraints,
                       const std::vector<bool>& fconstraints){
    bool rval = true;
    if(fconstraints.at(0)){
        rval = rval && constraints.at(0)->isSatisfied(current);
    }
    if(fconstraints.at(2)){
        rval = rval && constraints.at(2)->isSatisfied(current);
    }
    return rval;
}

int LocalPlannerClassic::transform2Odo(SubPath& waypoints, ros::Time& now){
    // calculate the corrective transformation to map from world coordinates to odom
    Stopwatch sw;
    sw.restart();
    if(!transformer_.waitForTransform("map", "odom", now, ros::Duration(0.1))) {
        ROS_WARN_THROTTLE_NAMED(1, "local_path", "cannot transform map to odom");
        return 0;
    }
    ROS_INFO_STREAM("Time Leak here: " << sw.usElapsed()/1000.0 << "ms");

    tf::StampedTransform now_map_to_odom;
    transformer_.lookupTransform("map", "odom", now, now_map_to_odom);

    tf::Transform transform_correction = now_map_to_odom.inverse();

    // transform the waypoints from world to odom
    for(Waypoint& wp : waypoints) {
        tf::Point pt(wp.x, wp.y, 0);
        pt = transform_correction * pt;
        wp.x = pt.x();
        wp.y = pt.y();

        tf::Quaternion rot = tf::createQuaternionFromYaw(wp.orientation);
        rot = transform_correction * rot;
        wp.orientation = tf::getYaw(rot);
    }
    return 1;
}

void LocalPlannerClassic::initConstraintsAndScorers(const std::vector<Constraint::Ptr>& constraints,
                                                    const std::vector<Scorer::Ptr>& scorer,
                                                    const std::vector<bool>& fconstraints,
                                                    const std::vector<double>& wscorer,
                                                    SubPath& waypoints){
    if(fconstraints.at(0)){
        std::dynamic_pointer_cast<Dis2Path_Constraint>(constraints.at(0))->setSubPath(waypoints,
                                                                                      index1, index2);
    }
    if(fconstraints.at(1)){
        if(last_local_path_.empty()){
            std::dynamic_pointer_cast<Dis2Path_Constraint>(constraints.at(1))->setSubPath(last_local_path_,
                                                                                      last_local_path_.size()-1, 0);
        }else{
            std::dynamic_pointer_cast<Dis2Path_Constraint>(constraints.at(1))->setSubPath(last_local_path_,
                                                                                      0, last_local_path_.size()-1);
        }
    }
    if(wscorer.at(0) != 0.0){
        std::dynamic_pointer_cast<Dis2Start_Scorer>(scorer.at(0))->setPath(waypoints, c_dist,
                                                                           index1, index2);
    }
    if(wscorer.at(1) != 0.0){
        std::dynamic_pointer_cast<Dis2Path_Scorer>(scorer.at(1))->setSubPath(waypoints,
                                                                             index1, index2);
    }
    if(wscorer.at(3) != 0.0){
        if(last_local_path_.empty()){
            std::dynamic_pointer_cast<Dis2Path_Scorer>(scorer.at(3))->setSubPath(last_local_path_,
                                                                             last_local_path_.size()-1, 0);
        }else{
            std::dynamic_pointer_cast<Dis2Path_Scorer>(scorer.at(3))->setSubPath(last_local_path_,
                                                                             0, last_local_path_.size()-1);
        }
    }
}

void LocalPlannerClassic::setPath(Path::Ptr& local_path, SubPath& local_wps, ros::Time& now){
    local_path->setPath({local_wps});

    follower_.getController()->reset();
    follower_.getController()->setPath(local_path);

    last_update_ = now;
}

void LocalPlannerClassic::smoothAndInterpolate(SubPath& local_wps){
    //smoothing
    local_wps = smoothPath(local_wps, 0.6, 0.15);
    //interpolate
    local_wps = interpolatePath(local_wps, 0.1);
    //final smoothing
    local_wps = smoothPath(local_wps, 2.0, 0.4);
}

void LocalPlannerClassic::printSCTimeUsage(const std::vector<Constraint::Ptr>& constraints,
                                           const std::vector<Scorer::Ptr>& scorer,
                                           const std::vector<bool>& fconstraints,
                                           const std::vector<double>& wscorer){
    for(std::size_t i = 0; i < constraints.size(); ++i){
        if(fconstraints.at(i)){
            ROS_INFO_STREAM("Constraint #" << (i+1) << " took " << constraints.at(i)->nsUsed()/1000.0 << " us");
        }
    }
    for(std::size_t i = 0; i < scorer.size(); ++i){
        if(wscorer.at(i) != 0.0){
            ROS_INFO_STREAM("Scorer #" << (i+1) << " took " << scorer.at(i)->nsUsed()/1000.0 << " us");
        }
    }
}

Path::Ptr LocalPlannerClassic::updateLocalPath(const std::vector<Constraint::Ptr>& constraints,
                                                   const std::vector<Scorer::Ptr>& scorer,
                                                   const std::vector<bool>& fconstraints,
                                                   const std::vector<double>& wscorer)
{
    ros::Time now = ros::Time::now();
    Stopwatch gsw;
    gsw.restart();

    if(last_update_ + update_interval_ < now) {

        // only look at the first sub path for now
        auto waypoints = (SubPath) global_path_;

        if(transform2Odo(waypoints,now) == 0){
            return nullptr;
        }
        Eigen::Vector3d pose = follower_.getRobotPose();
        int nnodes = 0;

        std::vector<Waypoint> local_wps;

        if(algo(pose, waypoints, local_wps, constraints, scorer, fconstraints, wscorer,
                nnodes) == 0){
            return nullptr;
        }

        Path::Ptr local_path(new Path("/odom"));
        setPath(local_path, local_wps, now);
        int end_t = gsw.usElapsed();

        printNodeUsage(nnodes);
        printSCTimeUsage(constraints, scorer, fconstraints, wscorer);
        ROS_INFO_STREAM("Local Planner duration: " << (end_t/1000.0) << " ms");

        return local_path;

    } else {
        return nullptr;
    }
}
