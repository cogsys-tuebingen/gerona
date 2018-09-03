// HEADER
#include <path_follower/controller/robotcontroller_modelbased.h>

// THIRD PARTY
#include <tf/tf.h>
#include <cv_bridge/cv_bridge.h>

// PROJECT
#include <path_follower/utils/pose_tracker.h>
#include <path_follower/utils/obstacle_cloud.h>
#include <path_follower/parameters/path_follower_parameters.h>
#include <path_follower/collision_avoidance/collision_avoider.h>
#include <cslibs_utils/MathHelper.h>
#include <path_follower/factory/controller_factory.h>
#include <cslibs_utils/Stopwatch.h>
// SYSTEM
#include <boost/algorithm/clamp.hpp>




#ifdef MODEL_CONTROLLER_DEBUG
#include <cv_bridge/cv_bridge.h>
#include <utils_draw.h>

#endif


REGISTER_ROBOT_CONTROLLER(RobotController_ModelBased, modelbased, default_collision_avoider);

RobotController_ModelBased::RobotController_ModelBased():
    RobotController(),
    cmd_(this)
{
    subDEM_ = nh_.subscribe ("/elevation_map", 1, &RobotController_ModelBased::imageCallback, this);

    initialized_ = false;
    doPlan_ = false;
    localMapFrame_ = PathFollowerParameters::getInstance()->robot_frame();;
    commandStatus = MBC_CommandStatus::OKAY;


    totalPlanningTime_ = 0;
    maxPlanningTime_ = 0;
    frameCounter_ = 0;


}


void RobotController_ModelBased::stopMotion()
{

    MoveCommand mcmd(true);
    mcmd.setDirection(0);
    mcmd.setVelocity(0);
    mcmd.setRotationalVelocity(0);
    publishMoveCommand(mcmd);

    cmd_.direction_angle = 0;
    cmd_.rotation = 0;
    cmd_.speed = 0;

    doPlan_ = false;

}


void RobotController_ModelBased::initialize()
{
    RobotController::initialize();

    transformer_ = &pose_tracker_->getTransformListener();

    m_opt_.AssignParams(config);

    config.expanderConfig_.minLinVel = opt_.min_linear_velocity();
    config.expanderConfig_.maxLinVel = opt_.max_linear_velocity();
    config.expanderConfig_.maxAngVel = opt_.max_angular_velocity();


    config.Setup();

    if (!initialized_)
    {

        if (!config.ReadRobotDescription(m_opt_.robot_config_file()))
        {
            ROS_ERROR_STREAM_THROTTLE(1, "Error reading robot description: " << m_opt_.robot_config_file());
            return;
        }
        if (!config.ReadMapDescription(m_opt_.elevation_map_config_file()))
        {
            ROS_ERROR_STREAM_THROTTLE(1, "Error reading elevation map description: " << m_opt_.elevation_map_config_file());
            return;
        }

        //config.wheelsConfig_.wheelPosRobotRearY*2.0;

        model_based_planner_ = IModelBasedPlanner::Create(config);
        if (!model_based_planner_)
        {
            ROS_ERROR_STREAM_THROTTLE(1, "Error Creating model based planner instance. ");
            return;
        }
        initialized_ = true;

        cv::Point3f n_pose(0,0,0);
        model_based_planner_->SetRobotPose(n_pose);


#ifdef MODEL_CONTROLLER_DEBUG
        dbgImgPub_ =     nh_.advertise<sensor_msgs::Image>("/model_dbg_image",1);

#endif

    }

    model_based_planner_->SetPlannerParameters(config.plannerConfig_);
    model_based_planner_->SetPlannerExpanderParameters(config.expanderConfig_);
    model_based_planner_->SetPlannerScorerParameters(config.scorerConfig_);


    doPlan_ = true;
    commandStatus = MBC_CommandStatus::OKAY;



    // desired velocity
    //vn_ = std::min(PathFollowerParameters::getInstance()->max_velocity(), velocity_);
    ROS_DEBUG_STREAM("Initialize ModelBased Controller...");
}


void RobotController_ModelBased::start()
{
    RobotController::start();
#ifdef MODEL_CONTROLLER_DEBUG
    if (opt_.pose_output_folder().length() > 2)
    {
        writer_.Init(opt_.pose_output_folder());
        writer_.WriteConfig(config,m_opt_.robot_config_file());
    }
#endif
}

void RobotController_ModelBased::reset()
{
    RobotController::reset();
}


/*
void RobotController_ModelBased::setPath(Path::Ptr path)
{
    RobotController::setPath(path);
    path_interpol.get_end(goal_);
}
*/

/*
void RobotController_ModelBased::setGoalPosition()
{


}
*/

bool RobotController_ModelBased::CheckNextPath()
{


    path_->switchToNextSubPath();
    // check if we reached the actual goal or just the end of a subpath
    if (path_->isDone()) {

        /*
        MoveCommand cmd_stop(true);
        cmd_stop.setVelocity(0.0);
        cmd_stop.setDirection(0.0);
        cmd_stop.setRotationalVelocity(0.0);

        *cmd = cmd_stop;
        * */

        Eigen::Vector3d current_pose = pose_tracker_->getRobotPose();
        double x_meas = current_pose[0];
        double y_meas = current_pose[1];


        double distance_to_goal_eucl = hypot(x_meas - path_interpol.p(path_interpol.n()-1),
                                             y_meas - path_interpol.q(path_interpol.n()-1));

        ROS_INFO_THROTTLE(1, "Final positioning error: %f m", distance_to_goal_eucl);
        return true;

    } else {
        //reset the orthogonal projection

        ROS_INFO("Next subpath...");
        // interpolate the next subpath
        path_interpol.interpolatePath(path_);
        publishInterpolatedPath();


        // recompute the driving direction
        computeMovingDirection();

        path_interpol.get_end(goal_);

        return false;
    }

    return false;
}

RobotController::MoveCommandStatus RobotController_ModelBased::computeMoveCommand(MoveCommand *cmd)
{
    *cmd = cmd_;

    ros::Time now = ros::Time::now();

    //RobotController::findOrthogonalProjection();



    if (commandStatus == MBC_CommandStatus::REACHED_GOAL )
    {
        ROS_INFO_THROTTLE(1, "MBC: Reached Goal!");
        return MoveCommandStatus::REACHED_GOAL;
    }

    if(commandStatus != MBC_CommandStatus::OKAY)
    {
        ROS_WARN_THROTTLE(1, "MBC: no valid path found!");
        return MoveCommandStatus::ERROR;
    }


    /*
    if(RobotController::isGoalReached(cmd)){
        return RobotController::MoveCommandStatus::REACHED_GOAL;
    }
    */


    path_interpol.get_end(goal_);

    if(!targetTransform2base(now)){
        ROS_WARN_THROTTLE(1, "MBC: cannot transform goal! World to Odom not known!");
        return MoveCommandStatus::ERROR;
    }

    cv::Point3f goal(target_.x,target_.y,target_.orientation);

    model_based_planner_->SetGoalMap(goal);
    model_based_planner_->SetPathMap(currentPath_);

    doPlan_ = true;

    return MoveCommandStatus::OKAY;
}



void RobotController_ModelBased::publishMoveCommand(const MoveCommand &cmd) const
{
    geometry_msgs::Twist msg;
    msg.linear.x  = cmd.getVelocity();
    msg.linear.y  = 0;
    msg.angular.z = cmd.getRotationalVelocity();

    cmd_pub_.publish(msg);
}


bool RobotController_ModelBased::GetTransform(ros::Time time,std::string targetFrame, std::string sourceFrame, tf::StampedTransform &trans)
{
    try{//Try to get the latest avaiable Transform
        transformer_->lookupTransform(targetFrame, sourceFrame, time, trans);
    }catch(tf::TransformException ex){//if not available, then wait
        (void) ex;
        if(!transformer_->waitForTransform(targetFrame, sourceFrame, time, ros::Duration(0.05))){
            ROS_WARN_THROTTLE_NAMED(1, "local_path", "cannot transform map to odom");
            return false;
        }
        transformer_->lookupTransform(targetFrame, sourceFrame, time, trans);
    }

    return true;


}


void RobotController_ModelBased::TransformPath(tf::Transform trans)
{
    currentPath_.clear();
    for (unsigned int tl = 0; tl < path_interpol.n();++tl)
    {
        cv::Point3f resP;
        tf::Point pt(path_interpol.p(tl), path_interpol.q(tl), 0);
        pt = trans * pt;
        resP.x = pt.x();
        resP.y = pt.y();

        tf::Quaternion rot = tf::createQuaternionFromYaw(path_interpol.s(tl));
        rot = trans * rot;
        resP.z = tf::getYaw(rot);

        currentPath_.push_back(resP);
    }


}


bool RobotController_ModelBased::targetTransform2base(ros::Time& now)
{
    tf::StampedTransform now_map_to_base;

    std::string world_frame = PathFollowerParameters::getInstance()->world_frame();
    std::string robot_frame = localMapFrame_;

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

    tf::Point pt(goal_.x, goal_.y, 0);
    pt = transform_correction * pt;
    target_.x = pt.x();
    target_.y = pt.y();

    tf::Quaternion rot = tf::createQuaternionFromYaw(goal_.orientation);
    rot = transform_correction * rot;
    target_.orientation = tf::getYaw(rot);

    TransformPath(transform_correction);
    return true;
}



void RobotController_ModelBased::imageCallback (const sensor_msgs::ImageConstPtr& image)
{
    //ROS_DEBUG_STREAM_THROTTLE(1.0,"Received DEM Image...");

    localMapFrame_ = image->header.frame_id;

    if (!initialized_)
    {
        ROS_WARN_STREAM("ModelBased Controller not initialized!");
        return;

    }

    ros::Time mapTime = image->header.stamp;


    if (!doPlan_) return;

    std::string robot_frame = PathFollowerParameters::getInstance()->robot_frame();
    std::string map_frame = image->header.frame_id;

    cv::Mat inputImage = cv_bridge::toCvShare(image,"")->image;

    cv::Point3f pose(0,0,0);


    if (map_frame != robot_frame)
    {
        tf::Transform transLocalMap =pose_tracker_->getTransform(map_frame ,localMapFrame_,mapTime,ros::Duration(0.04));
        /*
        tf::StampedTransform transLocalMap;
        if (!GetTransform(now, map_frame, localMapFrame_, transLocalMap))
        {
            ROS_ERROR_STREAM_THROTTLE(1, "RobotController_ModelBased: Cannot transform local map to map!" );

            return;
        }
        */

        model_based_planner_->SetDEMPos(cv::Point2f(transLocalMap.getOrigin().x(), transLocalMap.getOrigin().y()));


        /*
        tf::StampedTransform trans;
        if (!GetTransform(now, map_frame, robot_frame, trans))
        {
            ROS_ERROR_STREAM_THROTTLE(1, "RobotController_ModelBased: Cannot transform local map to map!" );

            return;
        }
        */
        ros::Time now = ros::Time::now();
        tf::Transform trans =pose_tracker_->getTransform(map_frame ,robot_frame,now,ros::Duration(0.04));


        
        pose.x = trans.getOrigin().x();
        pose.y = trans.getOrigin().y();
        tf::Matrix3x3 rotMat(trans.getRotation());
        double roll,pitch,yaw;
        rotMat.getRPY(roll,pitch,yaw);
        pose.z = yaw;

        model_based_planner_->SetRobotPose(pose);

    }

    if (!opt_.use_lin_velocity() && !opt_.use_ang_velocity())
    {
        cv::Point2f vel(opt_.max_linear_velocity(),0);
        model_based_planner_->SetVelocity(vel);
    }
    else
    {
        geometry_msgs::Twist gVel = pose_tracker_->getVelocity();
        cv::Point2f nvel(gVel.linear.x,gVel.angular.z);
        if (!opt_.use_ang_velocity()) nvel.y = 0;
        if (!opt_.use_lin_velocity()) nvel.x = opt_.max_linear_velocity();

        // Testing

        if (config.expanderConfig_.firstLevelLinearSplits == 0)
        {
            nvel.x += opt_.lin_acc_step();
        }


        if (nvel.x > opt_.max_linear_velocity()) nvel.x = opt_.max_linear_velocity();

        if (opt_.k_g() > 0 && opt_.lin_acc_step() > 0)
        {
            cv::Point3f tgoal = currentPath_.back();
            cv::Point3f diff = tgoal-pose;
            float distanceToGoal = sqrt( diff.x*diff.x + diff.y*diff.y);

            if (distanceToGoal < opt_.k_g())
            {
                nvel.x -= opt_.lin_acc_step();
            }

            if (nvel.x < opt_.min_linear_velocity()) nvel.x = opt_.min_linear_velocity();
        }

        //

        model_based_planner_->SetVelocity(nvel);
    }

    model_based_planner_->UpdateDEM(inputImage);

    Stopwatch sw;
    sw.restart();

    model_based_planner_->Plan();

    double curMS = sw.usElapsed()/1000.0;
    totalPlanningTime_ += curMS;
    frameCounter_++;
    if (curMS > maxPlanningTime_)maxPlanningTime_ = curMS;

    //ROS_INFO_STREAM_THROTTLE(1,"Model based planner took " << totalPlanningTime_/(double)frameCounter_ << "ms");
    ROS_INFO_STREAM_THROTTLE(1,"Model based planner took " << curMS << " Avg: " << totalPlanningTime_/(double)frameCounter_ << "ms" << " Frames: " << frameCounter_);

    Trajectory *result = model_based_planner_->GetBLResultTrajectory();


#ifdef MODEL_CONTROLLER_DEBUG
    if (dbgImgPub_.getNumSubscribers() > 0)
    {
        sw.restart();

        cv::Mat debugImg = model_based_planner_->DrawDebugImage(1.0f,true);

        cv_bridge::CvImage out_dbg_image;
        out_dbg_image.header   = image->header; // Same timestamp and tf frame as input image
        out_dbg_image.encoding = sensor_msgs::image_encodings::RGB8; // Or whatever
        out_dbg_image.image    = debugImg; // Your cv::Mat
        dbgImgPub_.publish(out_dbg_image.toImageMsg());

        ROS_INFO_STREAM_THROTTLE(1,"Draw debug image took " << sw.usElapsed()/1000.0 << "ms");

    }

    if (result != nullptr && result->end_ != nullptr)
    {
        ROS_INFO_STREAM("End State: " << result->end_->validState << " : " << PoseEvalResults::GetValidStateString(result->end_->validState));

    }

    //writer_.WritePoses(model_based_planner_->GetBLResultTrajectory(),model_based_planner_->GetDEMPos());
    if (opt_.pose_output_folder().length() > 2)
    {
        //writer_.WritePoses(model_based_planner_->GetBLResultTrajectory(),inputImage,model_based_planner_->GetDEMPos());
        writer_.WritePoses(model_based_planner_->GetBLResultTrajectory(),model_based_planner_->GetDEMPos(), pose);
        writer_.WriteTimings((float)curMS, model_based_planner_->GetPoseCount());
    }
#endif





    if (result == nullptr)
    {
        ROS_WARN_STREAM("Model based controller: No valid trajectory found! ");
        commandStatus = MBC_CommandStatus::ERROR;
        stopMotion();
        return;

    }

    if (result->end_ == nullptr)
    {
        ROS_WARN_STREAM("Model based controller: No valid trajectory found! ");
        commandStatus = MBC_CommandStatus::ERROR;
        stopMotion();
        return;

    }

    bool reachedGoal = result->end_->validState == PERS_GOALREACHED;


    if (result->poseResults_.size() < 2)
    {

        ROS_WARN_STREAM("Model based controller: Result trajectory only contains current position! #poses: " << result->poseResults_.size() );
        //commandStatus = reachedGoal? MBC_CommandStatus::REACHED_GOAL : MBC_CommandStatus::COLLISON;
        commandStatus = MBC_CommandStatus::COLLISON;
        stopMotion();
        return;
    }

    /*
    if ((int)result->poseResults_.size() < opt_.min_traj_nodes() && (reachedGoal) )
    {
        ROS_WARN_STREAM("Model based controller: Result trajectory to short: " << result->poseResults_.size() << " Min: " << opt_.min_traj_nodes());
        //commandStatus = reachedGoal? MBC_CommandStatus::REACHED_GOAL : MBC_CommandStatus::COLLISON;
        commandStatus = MBC_CommandStatus::REACHED_GOAL;
        stopMotion();
        return;
    }
    */

    if ((int)result->poseResults_.size() < opt_.min_traj_nodes() && (!reachedGoal) )
    {
        ROS_WARN_STREAM("Model based controller: Result trajectory to short: " << result->poseResults_.size() << " Min: " << opt_.min_traj_nodes());
        //commandStatus = reachedGoal? MBC_CommandStatus::REACHED_GOAL : MBC_CommandStatus::COLLISON;
        commandStatus = MBC_CommandStatus::COLLISON;
        stopMotion();
        return;
    }

    if ((int)result->poseResults_.size() < opt_.min_traj_nodes_goal() && (reachedGoal) )
    {
        bool lastPath = CheckNextPath();

        if (lastPath)
        {
            ROS_INFO_STREAM("Model based controller: Remaining Poses to Goal: " << result->poseResults_.size() << " Min: " << opt_.min_traj_nodes_goal());
            commandStatus = MBC_CommandStatus::REACHED_GOAL;
            //commandStatus = MBC_CommandStatus::COLLISON;
            stopMotion();
            return;
        }
    }

    /*
    if ((int)result->poseResults_.size() < opt_.min_traj_nodes() && reachedGoal)
    {
        ROS_INFO_STREAM("Model based controller: Reached Goal!");
        commandStatus = MBC_CommandStatus::REACHED_GOAL;
        cmd_.speed = 0;
        cmd_.rotation = 0;
        cmd_.direction_angle = 0;
        publishMoveCommand(cmd_);

        return;
    }*/




    cv::Point2f resCmd = result->poseResults_[1].cmd;
    cv::Point3f respose = result->poseResults_[1].pose;

    cmd_.speed = resCmd.x;
    cmd_.rotation = resCmd.y;
    cmd_.direction_angle = atan2(respose.y,respose.x);

    commandStatus = MBC_CommandStatus::OKAY;


    publishMoveCommand(cmd_);





}
