/// HEADER
#include <path_follower/local_planner/local_planner_implemented.h>

/// PROJECT

#include <path_follower/controller/robotcontroller.h>
#include <path_follower/utils/pose_tracker.h>

LocalPlannerImplemented::LocalPlannerImplemented()
    : waypoints(), wlp_(), tooClose(false), last_update_(0)
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
        transformer_->lookupTransform("map", "odom", ros::Time(0), now_map_to_odom);
    }catch(tf::TransformException ex){//If not avaiable, then wait
        (void) ex;
        if(!transformer_->waitForTransform("map", "odom", now, ros::Duration(0.1))){
            ROS_WARN_THROTTLE_NAMED(1, "local_path", "cannot transform map to odom");
            return false;
        }
        transformer_->lookupTransform("map", "odom", now, now_map_to_odom);
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

    controller_->reset();

    if(local_wps.empty()) {
        local_path->setPath({});
        controller_->stopMotion();

    } else {
        local_path->setPath({local_wps});
        controller_->setPath(local_path);
    }

    last_update_ = now;
}

void LocalPlannerImplemented::printSCTimeUsage(){
    for(std::size_t i = 0; i < constraints.size(); ++i){
        ROS_INFO_STREAM("Constraint #" << (i+1) << " took " << constraints.at(i)->nsUsed()/1000.0 << " us");
    }
    for(std::size_t i = 0; i < scorers.size(); ++i){
        ROS_INFO_STREAM("Scorer #" << (i+1) << " took " << scorers.at(i)->nsUsed()/1000.0 << " us");
    }
}

Path::Ptr LocalPlannerImplemented::updateLocalPath(Path::Ptr& wlp)
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

        Path::Ptr local_path(new Path("odom"));

        if(!transform2Odo(now)){
            ROS_WARN_THROTTLE(1, "cannot calculate local path, transform to odom not known");
            return local_path;
        }

        if(!obstacle_cloud_) {
            ROS_WARN_THROTTLE(1, "cannot calculate local path, no obstacle cloud received yet");
            return local_path;
        }

        /*
        ofstream myfile;
        myfile.open ("/tmp/pose.txt");
        */
        Eigen::Vector3d pose = pose_tracker_->getRobotPose();
        /*
        myfile << pose(0) << ", " << pose(1) << ", " << pose(2)<< std::endl;
        myfile.close();
        */
        std::size_t nnodes = 0;

        SubPath local_wps;
        local_wps.forward = true;

        if(!algo(pose, local_wps, nnodes)){
            if(!wlp_.empty()){
                wlp->setPath({wlp_});
            }

            return local_path;
        }

        setPath(local_path, wlp, local_wps, now);
        int end_t = gsw.usElapsed();

        printVelocity();
        printNodeUsage(nnodes);
        printLevelReached();
        printSCTimeUsage();

        ROS_INFO_STREAM("Local Planner duration: " << (end_t/1000.0) << " ms");

        return local_path;

    } else {
        return nullptr;
    }
}
