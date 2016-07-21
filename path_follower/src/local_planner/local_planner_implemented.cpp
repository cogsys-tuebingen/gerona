/// HEADER
#include <path_follower/local_planner/local_planner_implemented.h>

/// PROJECT
#include <path_follower/pathfollower.h>

LocalPlannerImplemented::LocalPlannerImplemented(PathFollower &follower,
                                 tf::Transformer& transformer,
                                 const ros::Duration& update_interval)
    : LocalPlanner(follower, transformer), last_update_(0), update_interval_(update_interval),
      waypoints(), tooClose(false)
{

}

void LocalPlannerImplemented::setGlobalPath(Path::Ptr path)
{
    LocalPlanner::setGlobalPath(path);
    tooClose = false;
}

bool LocalPlannerImplemented::transform2Odo(ros::Time& now){
    // calculate the corrective transformation to map from world coordinates to odom
    Stopwatch sw;
    sw.restart();
    if(!transformer_.waitForTransform("map", "odom", now, ros::Duration(0.1))) {
        ROS_WARN_THROTTLE_NAMED(1, "local_path", "cannot transform map to odom");
        return false;
    }
    ROS_INFO_STREAM("Time Leak here: " << sw.usElapsed()/1000.0 << "ms");

    tf::StampedTransform now_map_to_odom;
    transformer_.lookupTransform("map", "odom", now, now_map_to_odom);

    tf::Transform transform_correction = now_map_to_odom.inverse();
    /*
    ofstream myfile;
    myfile.open ("/tmp/path.txt");
    */

    // transform the waypoints from world to odom
    for(Waypoint& wp : waypoints) {
        tf::Point pt(wp.x, wp.y, 0);
        pt = transform_correction * pt;
        wp.x = pt.x();
        wp.y = pt.y();

        tf::Quaternion rot = tf::createQuaternionFromYaw(wp.orientation);
        rot = transform_correction * rot;
        wp.orientation = tf::getYaw(rot);
        /*
        myfile << wp.x << ", " << wp.y << ", " << wp.orientation <<std::endl;
        */
    }
    /*
    myfile.close();
    */
    return true;
}

void LocalPlannerImplemented::setPath(Path::Ptr& local_path, SubPath& local_wps, ros::Time& now){
    local_path->setPath({local_wps});

    follower_.getController()->reset();
    follower_.getController()->setPath(local_path);

    last_update_ = now;
}

void LocalPlannerImplemented::printSCTimeUsage(const std::vector<Constraint::Ptr>& constraints,
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

Path::Ptr LocalPlannerImplemented::updateLocalPath(const std::vector<Constraint::Ptr>& constraints,
                                                   const std::vector<Scorer::Ptr>& scorer,
                                                   const std::vector<bool>& fconstraints,
                                                   const std::vector<double>& wscorer)
{
    ros::Time now = ros::Time::now();
    Stopwatch gsw;
    gsw.restart();

    if(last_update_ + update_interval_ < now && !tooClose) {

        // only look at the first sub path for now
        waypoints = (SubPath) global_path_;

        if(!transform2Odo(now)){
            return nullptr;
        }
        /*
        ofstream myfile;
        myfile.open ("/tmp/pose.txt");
        */
        Eigen::Vector3d pose = follower_.getRobotPose();
        /*
        myfile << pose(0) << ", " << pose(1) << ", " << pose(2)<< std::endl;
        myfile.close();
        */
        int nnodes = 0;

        std::vector<Waypoint> local_wps;

        if(!algo(pose, local_wps, constraints, scorer, fconstraints, wscorer, nnodes)){
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
