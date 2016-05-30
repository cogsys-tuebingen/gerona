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
    // this planner uses the Breadth-first search algorithm

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
        std::dynamic_pointer_cast<Dis2Path_Constraint>(constraints.at(1))->setSubPath(last_local_path_);
        std::dynamic_pointer_cast<Dis2Start_Scorer>(scorer.at(0))->setDistances(waypoints);
        std::dynamic_pointer_cast<Dis2Path_Scorer>(scorer.at(1))->setSubPath(waypoints);
        std::dynamic_pointer_cast<Dis2Path_Scorer>(scorer.at(3))->setSubPath(last_local_path_);

        // find the subpath that starts closest to the robot
        Eigen::Vector3d pose = follower_.getRobotPose();

        LNode wpose(pose(0),pose(1),pose(2),-1,0);


        const Waypoint& last = waypoints[waypoints.size()-1];
        const tf::Point lastp(last.x,last.y,last.orientation);
        const tf::Point wposep(pose(0),pose(1),pose(2));

        float dis2last = scorer.at(0)->score(lastp);

        if((dis2last - scorer.at(0)->score(wposep)) < 0.8){
            return nullptr;
        }

        std::vector<LNode> nodes;
        nodes.push_back(wpose);

        std::queue<int> fifo_i;
        fifo_i.push(0);
        double go_dist = std::numeric_limits<double>::infinity();
        int obj = -1;
        int li_level = 10;


        while(!fifo_i.empty() && nodes.at(fifo_i.empty()?nodes.size()-1:fifo_i.front()).level_ <= li_level){
            int cu_i = fifo_i.front();
            fifo_i.pop();
            LNode current = nodes.at(cu_i);
            if(isNearEnough(current,last)){
                obj = cu_i;
                break;
            }

            std::vector<int> successors;
            getSuccessors(current, cu_i, successors, nodes, constraints);
            for(std::size_t i = 0; i < successors.size(); ++i){
                const tf::Point processed(nodes[successors.at(i)].x, nodes[successors.at(i)].y,
                        nodes[successors.at(i)].orientation);
                double new_dist = (dis2last - scorer.at(0)->score(processed))
                        + scorer.at(1)->score(processed) + scorer.at(2)->score(processed)
                        + (constraints.at(1)->isSatisfied(processed)?scorer.at(3)->score(processed):0.0);
                if(new_dist < go_dist){
                    go_dist = new_dist;
                    obj = successors[i];
                }
                fifo_i.push(successors[i]);
            }
        }
        //ROS_INFO_STREAM("Reasons: " <<  !fifo_i.empty() << ", " << (cu_dist <= ldist) << ", "
        //                << (level.at(fifo_i.empty()?nodes.size()-1:fifo_i.front()) <= li_level));

        std::vector<Waypoint> local_wps;
        Stopwatch sw;
        if(obj != -1){
            int cu = obj;
            while(nodes.at(cu).parent_ != -1){
                local_wps.push_back(nodes.at(cu));
                cu = nodes.at(cu).parent_;
            }
            local_wps.push_back(nodes.at(cu));
            std::reverse(local_wps.begin(),local_wps.end());
            //smoothing
            sw.restart();
            local_wps = smoothPath(local_wps, 0.6, 0.15);
            //interpolate
            local_wps = interpolatePath(local_wps, 0.1);
            //final smoothing
            local_wps = smoothPath(local_wps, 2.0, 0.4);
            ROS_INFO_STREAM("Local path postprocessing took " << sw.usElapsed() << " us");
            last_local_path_.assign(local_wps.begin(),local_wps.end());
        }else{
            return nullptr;
        }

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

