/// HEADER
#include <path_follower/local_planner/local_planner.h>

//Now the compiler is happy
std::vector<double> LocalPlanner::DUMMY_VECTOR;

LocalPlanner::LocalPlanner(PathFollower &follower, tf::Transformer &transformer)
    : follower_(follower), transformer_(transformer), last_local_path_()
{

}

LocalPlanner::~LocalPlanner()
{

}

void LocalPlanner::setGlobalPath(Path::Ptr path)
{
    global_path_ = path;

    ros::Time now = ros::Time::now();

    if(transformer_.waitForTransform("map", "odom", now, ros::Duration(1.0))) {
        transformer_.lookupTransform("map", "odom", now, initial_map_to_odom_);
        return;
    }
    if(transformer_.waitForTransform("map", "odom", ros::Time(0), ros::Duration(1.0))) {
        ROS_WARN_NAMED("global_path", "cannot transform map to odom, using latest");
        transformer_.lookupTransform("map", "odom", ros::Time(0), initial_map_to_odom_);
        return;
    }

    ROS_ERROR_NAMED("global_path", "cannot transform map to odom");
}


//borrowed from path_planner/planner_node.cpp
SubPath LocalPlanner::interpolatePath(const SubPath& path, double max_distance){
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
void LocalPlanner::subdividePath(SubPath& result, Waypoint low, Waypoint up, double max_distance){
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
SubPath LocalPlanner::smoothPath(const SubPath& path, double weight_data, double weight_smooth, double tolerance){
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
std::vector<SubPath> LocalPlanner::segmentPath(const std::vector<Waypoint> &path){
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
SubPath LocalPlanner::smoothPathSegment(const SubPath& path, double weight_data, double weight_smooth, double tolerance){
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

void LocalPlanner::getSuccessors(const LNode& current, int index, std::vector<int>& successors,
                                 std::vector<LNode>& nodes, const std::vector<Constraint::Ptr>& constraints,
                                 std::vector<double>& g, std::vector<double>& f, bool repeat){
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
        const LNode succ(x,y,theta,index,current.level_+1);
        const tf::Point succp(x,y,theta);

        if(constraints.at(0)->isSatisfied(succp) && constraints.at(2)->isSatisfied(succp)){
            int wo = -1;
            if(!isInGraph(succ,nodes,wo)){
                nodes.push_back(succ);
                successors.push_back(nodes.size());
                if(repeat){
                    g.push_back(std::numeric_limits<double>::infinity());
                    f.push_back(std::numeric_limits<double>::infinity());
                }
            }else{
                if(repeat){
                    successors.push_back(wo);
                }
            }
        }
    }
}

bool LocalPlanner::isNearEnough(const Waypoint& current, const Waypoint& last){
    if(current.distanceTo(last) <= 0.05){
        return true;
    }
    return false;
}

bool LocalPlanner::isInGraph(const Waypoint& current, std::vector<LNode>& nodes, int& position){
    for(std::size_t i = 0; i < nodes.size(); ++i){
        double dis = current.distanceTo(nodes[i]);
        if(dis < 0.05){
            position = i;
            return true;
        }
    }
    return false;
}

bool LocalPlanner::isNull() const
{
    return false;
}
