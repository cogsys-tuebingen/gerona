/// HEADER
#include <path_follower/local_planner/local_planner_bfs.h>

/// PROJECT
#include <path_follower/pathfollower.h>

LocalPlannerBFS::LocalPlannerBFS(PathFollower &follower,
                                 tf::Transformer& transformer,
                                 const ros::Duration& update_interval)
    : LocalPlanner(follower, transformer), last_update_(0), update_interval_(update_interval)
{

}

void LocalPlannerBFS::setGlobalPath(Path::Ptr path)
{
    LocalPlanner::setGlobalPath(path);
}

Path::Ptr LocalPlannerBFS::updateLocalPath(const std::vector<Constraint::Ptr>& constraints,
                                                   const std::vector<Scorer::Ptr>& scorer)
{
    // this planner does not "plan" locally, but transforms the global path to the odometry frame
    // to eliminate odometry drift

    ros::Time now = ros::Time::now();

    // only calculate a new local path, if enough time has passed.
    // TODO: also replan for other reasons, e.g. the global path has changed, ...
    if(last_update_ + update_interval_ < now) {
        // only look at the first sub path for now
        auto waypoints = global_path_->getSubPath(0);

        // calculate the corrective transformation to map from world coordinates to odom
        if(!transformer_.waitForTransform("map", "odom", now, ros::Duration(0.1))) {
            ROS_WARN_THROTTLE_NAMED(1, "local_path", "cannot transform map to odom");
            return nullptr;
        }

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

        std::dynamic_pointer_cast<Dis2Path_Constraint>(constraints.at(0))->setSubPath(waypoints);

        // find the subpath that starts closest to the robot
        Eigen::Vector3d pose = follower_.getRobotPose();

        Waypoint wpose(pose(0),pose(1),pose(2));


        const Waypoint& last = waypoints[waypoints.size()-1];
        double ldist = wpose.distanceTo(last);

        std::vector<Waypoint> nodes;
        std::vector<int> parents;
        std::vector<int> level;
        nodes.push_back(wpose);
        parents.push_back(-1);
        level.push_back(0);

        std::queue<int> fifo_i;
        fifo_i.push(0);
        double cu_dist = 0.0;
        double go_dist = std::numeric_limits<double>::infinity();
        int obj = -1;
        int li_level = 8;

        while(!fifo_i.empty() && cu_dist <= ldist &&
              level.at(fifo_i.empty()?nodes.size()-1:fifo_i.front()) <= li_level){
            int c_index = fifo_i.front();
            fifo_i.pop();
            const Waypoint& current = nodes[c_index];
            if(isNearEnough(current,last)){
                obj = c_index;
                ROS_INFO_STREAM("Goal found: " << current.x << ", " << current.y);
                break;
            }

            std::vector<int> successors;
            getSuccessors(current, c_index, successors, nodes, parents, level, constraints);
            for(std::size_t i = 0; i < successors.size(); ++i){
                const Waypoint& processed = nodes[successors[i]];
                double new_dist = processed.distanceTo(last);
                if(new_dist < go_dist){
                    go_dist = new_dist;
                    obj = successors[i];
                    new_dist = wpose.distanceTo(processed);
                    if(new_dist > cu_dist){
                        cu_dist = new_dist;
                    }
                }
                fifo_i.push(successors[i]);
            }
        }
        ROS_INFO_STREAM("Reasons: " <<  !fifo_i.empty() << ", " << (cu_dist <= ldist) << ", "
                        << (level.at(fifo_i.empty()?nodes.size()-1:fifo_i.front()) <= li_level));

        std::vector<Waypoint> local_wps;
        if(obj != -1){
            int cu_i = obj;
            while(parents[cu_i] != -1){
                local_wps.push_back(nodes[cu_i]);
                cu_i = parents[cu_i];
            }
            local_wps.push_back(nodes[cu_i]);
            std::reverse(local_wps.begin(),local_wps.end());
        }else{
            return nullptr;
        }

        //        // example use of scorers
        //        // (return the path with the lowest score.)
        //        double score = 0;
        //        for(tf::Point& pt : local_path) {
        //            for(Scorer& scorer : scorers) {
        //                score += scorer.score(pt);
        //            }
        //        }

        //        // example use of constraints to check for goal conditions
        //        bool is_goal = true;
        //        tf::Point point = ....;
        //        for(Constraint& constraint : constraints) {
        //            if(!constraint.isSatisfied(point)) {
        //               is_goal = false;
        //               break;
        //            }
        //        }

        // here we just use the subpath without checking constraints / scorerers
        Path::Ptr local_path(new Path("/odom"));
        local_path->setPath({local_wps});

        follower_.getController()->reset();
        follower_.getController()->setPath(local_path);

        last_update_ = now;

        ROS_INFO_STREAM("Local Planner duration: " << ros::Time::now()-now << " s");

        return local_path;

    } else {
        return nullptr;
    }
}

void LocalPlannerBFS::getSuccessors(const Waypoint& current, int index, std::vector<int>& successors,
                                    std::vector<Waypoint>& nodes, std::vector<int>& parents,
                                    std::vector<int>& level, const std::vector<Constraint::Ptr>& constraints){
    successors.clear();
    for(int i = 0; i < 3; ++i){
        double theta;
        switch (i) {
        case 0:// straight
            theta = current.orientation;
            break;
        case 1:// right
            theta = current.orientation - D_THETA;
            break;
        case 2:// left
            theta = current.orientation + D_THETA;
            break;
        default:
            break;
        }
        double x = current.x + 0.15*std::cos(theta);
        double y = current.y + 0.15*std::sin(theta);
        const Waypoint succ(x,y,theta);
        const tf::Point succp(x,y,theta);
        if(!isNearEnough(current,succ) &&
                std::dynamic_pointer_cast<Dis2Path_Constraint>(constraints.at(0))->isSatisfied(succp)){
            successors.push_back(nodes.size());
            nodes.push_back(succ);
            parents.push_back(index);
            level.push_back(level[index]+1);
        }
    }
}

bool LocalPlannerBFS::isNearEnough(const Waypoint& current, const Waypoint& last){
    if(current.distanceTo(last) <= 0.05){
        return true;
    }
    return false;
}
