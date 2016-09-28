/// HEADER
#include <path_follower/local_planner/local_planner_implemented.h>

/// PROJECT
#include <path_follower/pathfollower.h>

LocalPlannerImplemented::LocalPlannerImplemented(PathFollower &follower,
                                 tf::Transformer& transformer,
                                 const ros::Duration& update_interval)
    : LocalPlanner(follower, transformer), update_interval_(update_interval),
      waypoints(), wlp_(), tooClose(false), last_update_(0)
{

}

void LocalPlannerImplemented::setGlobalPath(Path::Ptr path)
{
    LocalPlanner::setGlobalPath(path);
    tooClose = false;
}

bool LocalPlannerImplemented::transform2Odo(ros::Time& now){
    tf::StampedTransform now_map_to_odom;
    try{//Try to get the latest avaiable Transform
        transformer_.lookupTransform("map", "odom", ros::Time(0), now_map_to_odom);
    }catch(tf::TransformException ex){//If not avaiable, then wait
        (void) ex;
        if(!transformer_.waitForTransform("map", "odom", now, ros::Duration(0.1))){
            ROS_WARN_THROTTLE_NAMED(1, "local_path", "cannot transform map to odom");
            return false;
        }
        transformer_.lookupTransform("map", "odom", now, now_map_to_odom);
    }

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

void LocalPlannerImplemented::setPath(Path::Ptr& local_path, Path::Ptr& wlp, SubPath& local_wps, ros::Time& now){
    local_path->setPath({local_wps});
    if(!wlp_.empty()){
        wlp->setPath({wlp_});
    }

    follower_.getController()->reset();

    if(local_wps.empty()) {
        local_path->setPath({});
        follower_.getController()->stopMotion();

    } else {
        local_path->setPath({local_wps});
        follower_.getController()->setPath(local_path);
    }

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
                                                   const std::vector<double>& wscorer,
                                                   Path::Ptr& wlp)
{
    ros::Time now = ros::Time::now();
    Stopwatch gsw;
    gsw.restart();
    if(last_update_ + update_interval_ < now && !tooClose) {

        // only look at the first sub path for now
        waypoints_map = (SubPath) global_path_;
        waypoints = (SubPath) global_path_;
        wlp_.wps.clear();

//        for(const SubPath& sp : subpaths) {
//            const Waypoint& first = waypoints_map.at(0);
//            const Waypoint& second = waypoints_map.at(1);

//            Eigen::Vector2d first_p = first;
//            Eigen::Vector2d second_p = second;

//            Eigen::Vector2d dir = second_p - first_p;
//            double dir_angle = std::atan2(dir(1), dir(0));
//            double delta = MathHelper::AngleDelta(dir_angle, first.orientation);
//            bool forward = std::abs(delta) < M_PI / 2.0;

//            std::cout << (forward ? " > " : " < ");
//        }
//        std::cout << std::endl;

        if(!transform2Odo(now)){
            return nullptr;
        }

        if(!obstacle_cloud_) {
            ROS_WARN("cannot calculate local path, no obstacle cloud received yet");
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
        std::size_t nnodes = 0;

        SubPath local_wps;
        local_wps.forward = true;

        if(!algo(pose, local_wps, constraints, scorer, fconstraints, wscorer, nnodes)){
            if(!wlp_.empty()){
                wlp->setPath({wlp_});
            }
            return nullptr;
        }

        Path::Ptr local_path(new Path("/odom"));
        setPath(local_path, wlp, local_wps, now);
        int end_t = gsw.usElapsed();

        printVelocity();
        printNodeUsage(nnodes);
        printLevelReached();
        printSCTimeUsage(constraints, scorer, fconstraints, wscorer);

        ROS_INFO_STREAM("Local Planner duration: " << (end_t/1000.0) << " ms");

        return local_path;

    } else {
        return nullptr;
    }
}
