/// HEADER
#include <path_follower/local_planner/model_based/local_planner_model.h>

/// PROJECT
#include <path_follower/utils/pose_tracker.h>
#include <path_follower/parameters/path_follower_parameters.h>
#include <path_follower/factory/local_planner_factory.h>
#include <path_follower/utils/elevation_map.h>

#include <utils_draw.h>


#ifdef MODEL_PLANNER_DEBUG
#include <cv_bridge/cv_bridge.h>

#endif




REGISTER_LOCAL_PLANNER(LocalPlannerModel, HS_Model);


LocalPlannerModel::LocalPlannerModel()
{

    initialized_ = false;
    model_based_planner_ = nullptr;
    path_end_ = Waypoint(0,0,0);
    target_ = Waypoint(0,0,0);
    close_to_goal_ = false;

    numFrames_ = 0;
    totalPlanningTime_ = 0;


}


bool LocalPlannerModel::GetTransform(ros::Time time,std::string targetFrame, std::string sourceFrame, tf::StampedTransform &trans)
{
    try{//Try to get the latest avaiable Transform
        transformer_->lookupTransform(targetFrame, sourceFrame, time, trans);
    }catch(tf::TransformException ex){//if not available, then wait
        (void) ex;
        if(!transformer_->waitForTransform(targetFrame, sourceFrame, time, ros::Duration(0.05))){
            ROS_WARN_STREAM_THROTTLE_NAMED(1, "LocalPlannerModel", "cannot lookup transform from :" << targetFrame << " to " << sourceFrame);
            return false;
        }
        transformer_->lookupTransform(targetFrame, sourceFrame, time, trans);
    }

    return true;


}

void LocalPlannerModel::PublishDebugImage()
{
#ifdef MODEL_PLANNER_DEBUG
    if (dbgImgPub_.getNumSubscribers() > 0)
    {
        //DrawProc dp;
        //cv::Mat debugImg = dp.Draw(*model_based_planner_,elevation_map_->toCVMat(),procConfig);
        cv::Mat debugImg = model_based_planner_->DrawDebugImage(1.0f,true);


        cv_bridge::CvImage out_dbg_image;
        out_dbg_image.header   = elevation_map_->elevationMap->header; // Same timestamp and tf frame as input image
        out_dbg_image.encoding = sensor_msgs::image_encodings::RGB8; // Or whatever
        out_dbg_image.image    = debugImg; // Your cv::Mat
        dbgImgPub_.publish(out_dbg_image.toImageMsg());
    }
#endif
}

Path::Ptr LocalPlannerModel::updateLocalPath_BaseLink()
{
    ROS_WARN_STREAM_THROTTLE(1, "LocalPlannerModel: updateLocalPath_BaseLink  " << (elevation_map_ != nullptr));
    ros::Time now = ros::Time::now();
    Stopwatch gsw;
    gsw.restart();

    // only look at the first sub path for now
    //waypoints_map = (SubPath) global_path_;
    //waypoints = (SubPath) global_path_;
    //wlp_.wps.clear();


    std::string odom_frame = PathFollowerParameters::getInstance()->odom_frame();


    Path::Ptr local_path(new Path(odom_frame));

    //if (path_end_.x == 0 && path_end_.y == 0 && path_end_.orientation == 0) return local_path;

    if(!transform2base(now)){
        ROS_WARN_THROTTLE(1, "cannot calculate local path, transform to odom not known");
        return local_path;
    }


    cv::Point3f goal(target_.x,target_.y,target_.orientation);

    model_based_planner_->SetGoalMap(goal);


    const cv::Mat cvImage = elevation_map_->toCVMat();


    model_based_planner_->UpdateDEM(cvImage);

    //Eigen::Vector3d pose = pose_tracker_->getRobotPose();

    //std::size_t nnodes = 0;

    SubPath local_wps;
    local_wps.forward = true;

    if (!opt_->use_velocity())
    {
        cv::Point2f vel(opt_->max_linear_velocity(),0);
        model_based_planner_->SetVelocity(vel);
    }

    if(!algo(local_wps)){
        return local_path;
    }



    //std::string world_frame = PathFollowerParameters::getInstance()->world_frame();
    //std::string odom_frame = PathFollowerParameters::getInstance()->odom_frame();
    std::string robot_frame = PathFollowerParameters::getInstance()->robot_frame();

    PublishDebugImage();

    if(!transformWPS(robot_frame,odom_frame,local_wps,now )){
        ROS_WARN_THROTTLE(1, "cannot calculate local path, transform to odom not known");
        return local_path;
    }
    return setPath(odom_frame, local_wps, now);

    //return setPath(robot_frame, local_wps, now);


}

Path::Ptr LocalPlannerModel::CreateDummyWps()
{
    SubPath local_wps;
    local_wps.forward = true;

    for (unsigned int i = 0; i < 10;++i)
    {

        Waypoint wp;

        wp.x = (float)i;
        wp.y = 0;
        wp.orientation = 0;



        local_wps.push_back(wp);

    }
    std::string odom_frame = PathFollowerParameters::getInstance()->odom_frame();
    ros::Time now = ros::Time::now();

    return setPath(odom_frame, local_wps, now);

}

Path::Ptr LocalPlannerModel::updateLocalPath_LocalMap()
{


    ROS_WARN_STREAM_THROTTLE(1, "LocalPlannerModel: updateLocalPath_LocalMap " << (elevation_map_ != nullptr));
    ros::Time now = ros::Time::now();
    ros::Time mapTime = elevation_map_->getStamp();

    //Stopwatch gsw;
    //gsw.restart();


    std::string map_frame = PathFollowerParameters::getInstance()->world_frame();
    std::string odom_frame = PathFollowerParameters::getInstance()->odom_frame();
    std::string robot_frame = PathFollowerParameters::getInstance()->robot_frame();
    std::string local_frame = elevation_map_->getFrameId();


    Path::Ptr local_path(new Path(odom_frame));

    //return CreateDummyWps();


    //if (path_end_.x == 0 && path_end_.y == 0 && path_end_.orientation == 0) return local_path;


    tf::Transform transLocalMap = pose_tracker_->getTransform(map_frame, local_frame,mapTime,ros::Duration(0.01));

    /*
    tf::StampedTransform transLocalMap;
    if (!GetTransform(mapTime, map_frame, local_frame, transLocalMap))
    {
        ROS_ERROR_STREAM_THROTTLE(1, "LocalPlannerModel: Cannot transform local map to map!" );

        return local_path;
    }
    */

    model_based_planner_->SetDEMPos(cv::Point2f(transLocalMap.getOrigin().x(), transLocalMap.getOrigin().y()));


    //tf::Transform transform_correction = transLocalMap.inverse();

    //cv::Point3f goal(target_.x,target_.y,target_.orientation);
    cv::Point3f goal(path_end_.x, path_end_.y,path_end_.orientation);


    model_based_planner_->SetGoalMap(goal);
    model_based_planner_->SetPathMap(currentPath_);



    tf::Transform trans = pose_tracker_->getTransform(map_frame, robot_frame,mapTime,ros::Duration(0.01));

    /*
    tf::StampedTransform trans;
    if (!GetTransform(now, map_frame, robot_frame, trans))
    {
        ROS_ERROR_STREAM_THROTTLE(1, "LocalPlannerModel: Cannot transform local map to map!" );

        return local_path;
    }
    */

    cv::Point3f pose;

    pose.x = trans.getOrigin().x();
    pose.y = trans.getOrigin().y();
    tf::Matrix3x3 rotMat(trans.getRotation());
    double roll,pitch,yaw;
    rotMat.getRPY(roll,pitch,yaw);
    pose.z = yaw;

    cv::Point2f tGoalDist(path_end_.x-pose.x, path_end_.y-pose.y);

    if (sqrt(tGoalDist.dot(tGoalDist)) < opt_->min_distance_to_goal())
    {
        return local_path;
    }

    //Eigen::Vector3d pose = pose_tracker_->getRobotPose();

    model_based_planner_->SetRobotPose(pose);

    const cv::Mat cvImage = elevation_map_->toCVMat();


    model_based_planner_->UpdateDEM(cvImage);

    //Eigen::Vector3d pose = pose_tracker_->getRobotPose();

    //std::size_t nnodes = 0;

    SubPath local_wps;
    local_wps.forward = true;

    if (!opt_->use_velocity())
    {
        cv::Point2f vel(opt_->max_linear_velocity(),0);
        model_based_planner_->SetVelocity(vel);
    }

    if(!algo(local_wps)){
        return local_path;
    }

    PublishDebugImage();

    if(!transformWPS(map_frame,odom_frame,local_wps,now )){
        ROS_WARN_THROTTLE(1, "cannot calculate local path, transform to odom not known");
        return local_path;
    }
    return setPath(odom_frame, local_wps, now);

}


bool LocalPlannerModel::TestPath(SubPath &path)
{
    if (path.size() <= 1) return false;

    float dist = 0;


    for (unsigned int i = 1; i < path.size();++i)
    {
        float diffX = path[i].x - path[i-1].x;
        float diffY = path[i].y - path[i-1].y;

        dist += sqrt(diffX*diffX + diffY*diffY);
    }

    if (dist < opt_->safety_distance_forward()) return false;
    return true;



}


Path::Ptr LocalPlannerModel::updateLocalPath()
{
    if(!elevation_map_) {
        ROS_WARN_THROTTLE(1, "cannot compute local path without elevation_map_");
        return nullptr;
    }


    ros::Time now = ros::Time::now();
    if(last_update_ + update_interval_ < now && !close_to_goal_){
        //last_update_ = now;

        std::string elevationMap_frame = elevation_map_->getFrameId();
        std::string robot_frame = PathFollowerParameters::getInstance()->robot_frame();

        if (elevationMap_frame == robot_frame)
        {
            return updateLocalPath_BaseLink();
        }
        else
        {
            return updateLocalPath_LocalMap();
        }


    }
    else {
        return nullptr;
    }

}



void LocalPlannerModel::setVelocity(geometry_msgs::Twist vector)
{
    cv::Point2f cur_vel;

    cur_vel.x = vector.linear.x;
    cur_vel.y = vector.angular.z;

    //if (cur_vel.x < opt_->min_linear_velocity) cur_vel.x = lowerVelocity_;

    model_based_planner_->SetVelocity(cur_vel);

}

void LocalPlannerModel::setVelocity(geometry_msgs::Twist::_linear_type vector)
{
    cv::Point2f cur_vel;

    cur_vel.x = vector.x;
    cur_vel.y = 0;

    //if (cur_vel.x < lowerVelocity_) cur_vel.x = lowerVelocity_;

    model_based_planner_->SetVelocity(cur_vel);

}

void LocalPlannerModel::setVelocity(double velocity)
{
    cv::Point2f cur_vel;

    cur_vel.x = velocity;
    cur_vel.y = 0;

    //if (cur_vel.x < lowerVelocity_) cur_vel.x = lowerVelocity_;

    model_based_planner_->SetVelocity(cur_vel);
}

void LocalPlannerModel::reset()
{

}


void LocalPlannerModel::setParams(const LocalPlannerParameters& opt)
{

    //use_velocity_ = opt.use_velocity();
    //lowerVelocity_ = opt.min_linear_velocity();

    //ModelBasedPlannerConfig config;

    m_opt_.AssignParams(config_);

    config_.expanderConfig_.minLinVel = opt_->min_linear_velocity();
    config_.expanderConfig_.maxLinVel = opt_->max_linear_velocity();
    config_.expanderConfig_.maxAngVel = opt_->max_angular_velocity();


    config_.Setup();

    if (!initialized_)
    {

        if (!config_.ReadRobotDescription(m_opt_.robot_config_file()))
        {
            ROS_ERROR_STREAM_THROTTLE(1, "Error reading robot description: " << m_opt_.robot_config_file());
            return;
        }
        if (!config_.ReadMapDescription(m_opt_.elevation_map_config_file()))
        {
            ROS_ERROR_STREAM_THROTTLE(1, "Error reading elevation map description: " << m_opt_.elevation_map_config_file());
            return;
        }

        model_based_planner_ = IModelBasedPlanner::Create(config_);

        initialized_ = true;

        cv::Point3f n_pose(0,0,0);
        model_based_planner_->SetRobotPose(n_pose);


#ifdef MODEL_PLANNER_DEBUG
        ros::NodeHandle nh;
        dbgImgPub_ =     nh.advertise<sensor_msgs::Image>("/model_dbg_image",1);

#endif

    }

    model_based_planner_->SetPlannerParameters(config_.plannerConfig_);
    model_based_planner_->SetPlannerExpanderParameters(config_.expanderConfig_);
    model_based_planner_->SetPlannerScorerParameters(config_.scorerConfig_);





}


void LocalPlannerModel::setGlobalPath(Path::Ptr path)
{
    AbstractLocalPlanner::setGlobalPath(path);
    global_path_.get_end(path_end_);

    currentPath_.clear();
    currentPath_.reserve(global_path_.n());

    for (int tl = 0; tl < global_path_.n();++tl)
    {
        cv::Point3f p(global_path_.p(tl), global_path_.q(tl),global_path_.s(tl));
        currentPath_.push_back(p);
    }

    close_to_goal_ = false;

}


bool LocalPlannerModel::transform2base(ros::Time& now)
{
    tf::StampedTransform now_map_to_base;

    std::string world_frame = PathFollowerParameters::getInstance()->world_frame();
    std::string robot_frame = PathFollowerParameters::getInstance()->robot_frame();

    try{//Try to get the latest avaiable Transform
        transformer_->lookupTransform(world_frame, robot_frame, ros::Time(0), now_map_to_base);
    }catch(tf::TransformException ex){//if not available, then wait
        (void) ex;
        if(!transformer_->waitForTransform(world_frame, robot_frame, now, ros::Duration(0.1))){
            ROS_WARN_THROTTLE_NAMED(1, "local_path", "cannot transform map to odom");
            return false;
        }
        transformer_->lookupTransform(world_frame, robot_frame, now, now_map_to_base);
    }

    tf::Transform transform_correction = now_map_to_base.inverse();

    tf::Point pt(path_end_.x, path_end_.y, 0);
    pt = transform_correction * pt;
    target_.x = pt.x();
    target_.y = pt.y();

    tf::Quaternion rot = tf::createQuaternionFromYaw(path_end_.orientation);
    rot = transform_correction * rot;
    target_.orientation = tf::getYaw(rot);

    // transform the waypoints from world to odom
    /*
    for(Waypoint& wp : waypoints) {
        tf::Point pt(wp.x, wp.y, 0);
        pt = transform_correction * pt;
        wp.x = pt.x();
        wp.y = pt.y();

        tf::Quaternion rot = tf::createQuaternionFromYaw(wp.orientation);
        rot = transform_correction * rot;
        wp.orientation = tf::getYaw(rot);

    }
    */

    return true;
}



bool LocalPlannerModel::transformWPS(std::string source, std::string target, SubPath &waypoints, ros::Time& now)
{
    tf::StampedTransform now_transform;


    try{//Try to get the latest avaiable Transform
        transformer_->lookupTransform(target, source, ros::Time(0), now_transform);
    }catch(tf::TransformException ex){//if not available, then wait
        (void) ex;
        if(!transformer_->waitForTransform(target, source, now, ros::Duration(0.1))){
            ROS_WARN_THROTTLE_NAMED(1, "local_path", "cannot transform map to odom");
            return false;
        }
        transformer_->lookupTransform(target, source, now, now_transform);
    }

    tf::Transform transform_correction = now_transform;

    /*
    tf::Point pt(path_end_.x, path_end_.y, 0);
    pt = transform_correction * pt;
    target_.x = pt.x();
    target_.y = pt.y();

    tf::Quaternion rot = tf::createQuaternionFromYaw(path_end_.orientation);
    rot = transform_correction * rot;
    target_.orientation = tf::getYaw(rot);
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

    }


    return true;
}


void LocalPlannerModel::printTimeUsage()
{
    /*
    for(std::size_t i = 0; i < constraints.size(); ++i){
        ROS_INFO_STREAM("Constraint #" << (i+1) << " took " << constraints.at(i)->nsUsed()/1000.0 << " us");
    }
    for(std::size_t i = 0; i < scorers.size(); ++i){
        ROS_INFO_STREAM("Scorer #" << (i+1) << " took " << scorers.at(i)->nsUsed()/1000.0 << " us");
    }
    */

}

void LocalPlannerModel::printNodeUsage(std::size_t& nnodes) const
{

}

void LocalPlannerModel::printVelocity() const
{

}

void LocalPlannerModel::printLevelReached() const
{

}

bool LocalPlannerModel::algo(SubPath& local_wps)
{

    Stopwatch sw;
    sw.restart();

    model_based_planner_->Plan();

    double curMS = sw.usElapsed()/1000.0;
    totalPlanningTime_ += curMS;
    numFrames_++;

    //ROS_INFO_STREAM_THROTTLE(1,"Model based planner took " << totalPlanningTime_/(double)frameCounter_ << "ms");
    ROS_INFO_STREAM_THROTTLE(1,"Model based planner took " << curMS << " Avg: " << totalPlanningTime_/(double)numFrames_ << "ms" << " Frames: " << numFrames_);



    //model_based_planner_->Plan();


    Trajectory *result = model_based_planner_->GetBLResultTrajectory();

    if (result == nullptr) return false;

    if (result != nullptr && result->end_ != nullptr)
    {
        ROS_INFO_STREAM("End State: " << result->end_->validState << " : " << PoseEvalResults::GetValidStateString(result->end_->validState));

    }

    local_wps.wps.clear();

    PoseEvalResults *cr;

    for (unsigned int i = 0; i < result->poseResults_.size();++i)
    {
        cr = &result->poseResults_[i];

        Waypoint wp;

        wp.x = cr->pose.x;
        wp.y = cr->pose.y;
        wp.orientation = cr->pose.z;



        local_wps.push_back(wp);

    }

    bool reachedGoal = result->end_->validState == PERS_GOALREACHED;
    if (reachedGoal) return true;

    if (!TestPath(local_wps)) return false;

    return true;
}


