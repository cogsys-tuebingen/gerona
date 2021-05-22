// HEADER
#include <path_follower/controller/robotcontroller_velocity_TT.h>

// THIRD PARTY


// PROJECT
#include <path_follower/factory/controller_factory.h>
#include <path_follower/utils/pose_tracker.h>
#include <path_follower/collision_avoidance/collision_avoider.h>
#include <path_follower/utils/obstacle_cloud.h>


// SYSTEM
#include <cmath>
#include <opencv2/core/core.hpp>
#include <pcl_ros/transforms.h>


REGISTER_ROBOT_CONTROLLER(RobotController_Velocity_TT, velocity_TT, default_collision_avoider);

//using namespace Eigen;


RobotController_Velocity_TT::RobotController_Velocity_TT():
    RobotController()//,
  //cmd_(this)
{
    subTargetOdom_ = nh_.subscribe ("target_odom", 1, &RobotController_Velocity_TT::odomCallback, this);


}

void RobotController_Velocity_TT::stopMotion()
{

    //cmd_.vx = 0;
    //cmd_.vy = 0;
    //cmd_.rotation = 0;

    MoveCommand mcmd(true);// = cmd_;
    mcmd.setDirection(0);
    mcmd.setRotationalVelocity(0);
    mcmd.setVelocity(0);
    publishMoveCommand(mcmd);
}

void RobotController_Velocity_TT::initialize()
{
    RobotController::initialize();

    //reset the index of the current point on the path
    /*if (opt_.input_type() != VELTT_IT_PATH)
    {
        Path path(getFixedFrame());
        path.setFrameId(getFixedFrame());
        SubPath subPath;
        subPath.push_back(Waypoint(0,0,0));
        subPath.push_back(Waypoint(1000,0,0));
        subPath.forward = true;
        std::vector<SubPath> vecPath;
        vecPath.push_back(subPath);
        path.setPath(vecPath);
        path.setPath(vecPath);
        path.reset();
        Path::Ptr ptrPath = std::make_shared<Path>(path);
        setPath(ptrPath);
    }*/


}



void RobotController_Velocity_TT::start()
{
    RobotController::start();
}

void RobotController_Velocity_TT::reset()
{
    RobotController::reset();

}

void RobotController_Velocity_TT::setPath(Path::Ptr path)
{
    RobotController::setPath(path);

    if (opt_.input_type() == VELTT_IT_PATH)
    {
        Waypoint lastWP = path->getLastWaypoint();

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = path->getFrameId();
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = lastWP.x;
        pose.pose.position.y = lastWP.y;
        pose.pose.orientation.z = 1.0;
        processPose(pose);
    }


}

/*
double RobotController_EKM::computeSpeed()
{

}
*/
cv::Vec2f RobotController_Velocity_TT::CalcForceRep(const pcl::PointCloud<pcl::PointXYZ>& cloud, bool &hasObst)
{
    float min_dist = opt_.dist_thresh();
    float curDist = 0;
    float t1,krep = opt_.krep();
    float robot_radius = opt_.robot_radius();
    float min_dist_inv = 1.0f/min_dist;
    cv::Vec2f t2,tfrep;
    cv::Vec2f curP(0,0);
    cv::Vec2f sumF(0,0);
    int counter = 0;

    hasObst = false;

    for(const pcl::PointXYZ& pt : cloud) {
        curP[0] = pt.x;
        curP[1] = pt.y;

        curDist = std::sqrt(curP.dot(curP))-robot_radius;
        if(curDist < min_dist){

            t1 = (1.0f/curDist - min_dist_inv);
            t2 = (1.0f/(curDist*curDist))* ((-curP)*min_dist_inv);
            tfrep = krep*t1*t2;
            sumF = sumF+tfrep;
            counter++;
            hasObst = true;
        }

    }
    if (counter > 0) return sumF * (1.0f/(float)counter);
    else  return cv::Vec2f(0,0);

}

cv::Vec2f RobotController_Velocity_TT::CalcForceRep(bool &hasObst)
{
    auto obstacle_cloud = collision_avoider_->getObstacles();
    const pcl::PointCloud<pcl::PointXYZ>& cloud = *obstacle_cloud->cloud;

    if(cloud.header.frame_id == PathFollowerParameters::getInstance()->robot_frame()) {
        return CalcForceRep(cloud,hasObst);
    }
    else
    {
        if (!pose_tracker_->getTransformListener().waitForTransform(PathFollowerParameters::getInstance()->robot_frame(),
                                                                    obstacle_cloud->getFrameId(),
                                                                    obstacle_cloud->getStamp(),
                                                                    ros::Duration(0.1) ))
        {
            ROS_ERROR_THROTTLE_NAMED(1, "velocity_TT", "cannot transform cloud to robotframe");
            return cv::Vec2f(0,0);

        }
        pcl::PointCloud<pcl::PointXYZ> tcloud;

        pcl_ros::transformPointCloud(PathFollowerParameters::getInstance()->robot_frame(),
                                     cloud,
                                     tcloud,
                                     pose_tracker_->getTransformListener()
                                     );
        return CalcForceRep(tcloud,hasObst);
    }

}

cv::Vec2f RobotController_Velocity_TT::CalcForceAtt()
{
    if (targetPoses_.size() == 0) return cv::Vec2f(0,0);
    geometry_msgs::PoseStamped global = targetPoses_.back();
    global.header.stamp = ros::Time(0);
    geometry_msgs::PoseStamped local;
    local.pose.orientation.z = 1;
    if (!poseToLocalPose(global,local))
    {
        //ROS_WARN_THROTTLE(1, "Velocity_TT: CalcForceAtt cannot lookup tf target!");
        return cv::Vec2f(0,0);

    }

    cv::Vec2f goal(local.pose.position.x,local.pose.position.y);
    return -((float)opt_.katt())*(-goal);



}


geometry_msgs::PoseStamped RobotController_Velocity_TT::Odom2Pose(nav_msgs::Odometry odom)
{
    geometry_msgs::PoseStamped result;
    result.header = odom.header;
    result.pose = odom.pose.pose;
    return result;

}

float func6(float x)
{
    if (x < 0) return std::tan(x*( std::pow(CV_PI/2.0,std::abs(x)) ));
    else return std::tanh(x);
}


RobotController::MoveCommandStatus RobotController_Velocity_TT::computeMoveCommand(MoveCommand *cmd)
{
    if (opt_.input_type() == VELTT_IT_TF)
    {
        try
        {
            tf::Transform target_tf_world = pose_tracker_->getTransform(getFixedFrame(), "target", ros::Time::now(), ros::Duration(0.01));
            tf::Vector3 target_vec = target_tf_world.getOrigin();


            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = getFixedFrame();
            pose.header.stamp = ros::Time::now();
            pose.pose.position.x = target_vec.x();
            pose.pose.position.y = target_vec.y();

            processPose(pose);
        }
        catch(const tf::TransformException& ex)
        {
            ROS_WARN_THROTTLE(1, "Velocity_TT: cannot lookup tf target!");

        }
    }
    // omni drive can rotate.
    *cmd = MoveCommand(true);

    if (targetPoses_.size() == 0)
    {
        ROS_WARN_THROTTLE(1, "Velocity_TT: no target received!");
        cmd->setDirection(0);
        cmd->setVelocity(0);
        cmd->setRotationalVelocity(0);
        return MoveCommandStatus::OKAY;
    }


    bool hasObst = false;
    cv::Vec2f force = CalcForceAtt()+CalcForceRep(hasObst);


    Eigen::Vector3d ePose = pose_tracker_->getRobotPose();

    cv::Vec2f rPos(ePose[0],ePose[1]);

    geometry_msgs::PoseStamped tPose = targetPoses_.back();
    cv::Vec2f tPos(tPose.pose.position.x,tPose.pose.position.y);

    geometry_msgs::PoseStamped xDirLocal;
    xDirLocal.header.stamp = ros::Time(0);
    xDirLocal.header.frame_id = pose_tracker_->getRobotFrameId();
    xDirLocal.pose.position.x = 1.0;
    xDirLocal.pose.orientation.z = 1.0;
    geometry_msgs::PoseStamped xDirGlobal;



    if (!poseToGlobalPose(xDirLocal,xDirGlobal))
    {
        ROS_WARN_THROTTLE(1, "Velocity_TT: computeMoveCommand: error trans xdir!");
        cmd->setDirection(0);
        cmd->setVelocity(0);
        cmd->setRotationalVelocity(0);
        return MoveCommandStatus::OKAY;
    }

    cv::Vec2f xDir(xDirGlobal.pose.position.x,xDirGlobal.pose.position.y);

    float relTransporterVel = (lastTargetDir_.dot(xDir));

    cv::Vec2f r2t = tPos-rPos;

    float dist = std::sqrt(r2t.dot(r2t));

    float distDiff = dist - opt_.dest_distance();

    float velCorrection = func6(distDiff);

    float minScaleFact = opt_.min_scale_fact();

    float scaleFact = distDiff * opt_.dist_scale_fact();


    if (scaleFact < minScaleFact) scaleFact = minScaleFact;

    float u = relTransporterVel+velCorrection*scaleFact;

    float maxLinVel = PathFollowerParameters::getInstance()->max_velocity();

    if (u > maxLinVel) u = maxLinVel;
    if (u < -maxLinVel) u = -maxLinVel;


    geometry_msgs::PoseStamped tPoseLocal;


    if (!poseToLocalPose(tPose,tPoseLocal))
    {
        ROS_WARN_THROTTLE(1, "Velocity_TT: computeMoveCommand: error trans transporter pose!");
        cmd->setDirection(0);
        cmd->setVelocity(0);
        cmd->setRotationalVelocity(0);
        return MoveCommandStatus::OKAY;
    }
    cv::Vec2f local_r2t(tPoseLocal.pose.position.x,tPoseLocal.pose.position.y);

    float tw = std::atan2(local_r2t[1],local_r2t[0]);


    if (tw > CV_PI) tw = tw -2.0*CV_PI;
    if (tw < -CV_PI) tw = tw +2.0*CV_PI;

    float maxAngVel = opt_.max_angular_velocity();

    float omegaFact = opt_.kangular();

    float wres = std::tanh(tw*omegaFact)*maxAngVel;

    float vres = 0;

    float ures = u;
    if (std::abs(tw) > CV_PI/2.0 ) ures = 0;


    if (hasObst)
    {
        ures = force[0];
        vres = force[1];
    }

    if (ures > maxLinVel) ures = maxLinVel;
    if (ures < -maxLinVel) ures = -maxLinVel;
    if (vres > maxLinVel) vres = maxLinVel;
    if (vres < -maxLinVel) vres = -maxLinVel;


    ROS_INFO_STREAM("Velocity_TT1: tpos: " << tPos << " rpos: " << rPos << " r2t: " << r2t << " lr2t: " << local_r2t <<  " d: " << dist << " velcor: " << velCorrection );
    ROS_INFO_STREAM("Velocity_TT2: u:" << ures << " v: " << vres << " wres: " << wres << " tw: " << tw <<  "force: " << force << " hasobst: " << hasObst);

    /*
    cmd_.vx = ures;
    cmd_.vy = vres;
    cmd_.rotation = wres;

    *cmd = cmd_;
*/

    Eigen::Vector2f dir;
    dir[0] = ures;
    dir[1] = vres;

    cmd->setDirection(dir);
    cmd->setVelocity(dir.norm());

    if (ures == 0 && vres == 0)
    {
        cmd->setDirection(0);
        cmd->setVelocity(0);

    }


    cmd->setRotationalVelocity(wres);
    return MoveCommandStatus::OKAY;




}

void RobotController_Velocity_TT::publishMoveCommand(const MoveCommand &cmd) const
{
    geometry_msgs::Twist msg;
    msg.linear.x  = cmd.getVelocityVector()[0];
    msg.linear.y  = cmd.getVelocityVector()[1];
    msg.angular.z = cmd.getRotationalVelocity();

    cmd_pub_.publish(msg);
}

void TransformOdometry(const nav_msgs::Odometry &in_odom, nav_msgs::Odometry &out_odom, tf::TransformListener& tf_listener, std::string frame)
{
    geometry_msgs::PoseStamped in_pose;
    geometry_msgs::PoseStamped out_pose;
    in_pose.header = in_odom.header;
    in_pose.pose = in_odom.pose.pose;
    tf_listener.transformPose(frame,in_pose,out_pose);
    out_odom.pose.pose = out_pose.pose;

    geometry_msgs::Vector3Stamped in_vec,out_vec;
    in_vec.header = in_odom.header;
    in_vec.vector = in_odom.twist.twist.linear;
    tf_listener.transformVector(frame,in_vec,out_vec);
    out_odom.twist.twist.linear = out_vec.vector;
}


cv::Vec2f RobotController_Velocity_TT::estimateVel()
{
    if (targetPoses_.size() < 2) return cv::Vec2f(0,0);
    cv::Vec2f sumVels(0,0);
    float sumTime = 0;

    for (unsigned int tl = 1; tl < targetPoses_.size();++tl)
    {
        geometry_msgs::PoseStamped &o1 = targetPoses_[tl-1];
        cv::Vec2f v1(o1.pose.position.x,o1.pose.position.y);

        geometry_msgs::PoseStamped &o2 = targetPoses_[tl];
        cv::Vec2f v2(o2.pose.position.x,o2.pose.position.y);

        cv::Vec2f diff = v2-v1;

        sumVels += diff;
        sumTime += o2.header.stamp.toSec()-o1.header.stamp.toSec();
    }

    return sumVels * (1.0f/sumTime);
}


bool  RobotController_Velocity_TT::poseToGlobalPose(geometry_msgs::PoseStamped in_pose, geometry_msgs::PoseStamped &global)
{
    try {
        in_pose.header.stamp = ros::Time(0);
        pose_tracker_->getTransformListener().transformPose(pose_tracker_->getFixedFrameId(),in_pose,global);
        //pose_tracker_->getTransformListener().transformPose(in_pose.header.frame_id,ros::Time(0),in_pose,pose_tracker_->getFixedFrameId(),global);
        return true;

    } catch (tf::TransformException& ex) {
        ROS_ERROR("veltt: error with transform goal pose: %s", ex.what());
        return false;
    }
}

bool  RobotController_Velocity_TT::poseToLocalPose(geometry_msgs::PoseStamped in_pose, geometry_msgs::PoseStamped &local)
{
    try {
        in_pose.header.stamp = ros::Time(0);
        pose_tracker_->getTransformListener().transformPose(pose_tracker_->getRobotFrameId(),in_pose,local);
        //pose_tracker_->getTransformListener().transformPose(in_pose.header.frame_id,ros::Time(0),in_pose,pose_tracker_->getRobotFrameId(),local);
        //pose_tracker_->getTransformListener().transformPose(pose_tracker_->getRobotFrameId(),ros::Time(0),in_pose,in_pose.header.frame_id,local);
        return true;

    } catch (tf::TransformException& ex) {
        ROS_ERROR("veltt: error with transform goal pose: %s", ex.what());
        return false;
    }
}


void RobotController_Velocity_TT::processPose(const geometry_msgs::PoseStamped &pose)
{
    if (pose.header.frame_id != getFixedFrame())
    {
        geometry_msgs::PoseStamped tpose;
        //pose_tracker_->transformToGlobal(pose,tpose);
        if (!poseToGlobalPose(pose,tpose))
        {
            ROS_WARN_STREAM("VelTT: Error odom trans: from: " << pose.header.frame_id << " to " << getFixedFrame());
            return;
        }
        ROS_INFO_STREAM("VelTT: odom trans: from: " << pose.header.frame_id << " to " << getFixedFrame()<<  " x: " << tpose.pose.position.x << " y: " << tpose.pose.position.y );

        targetPoses_.push_back(tpose);
    }
    else
    {
        targetPoses_.push_back(pose);
    }
    std::size_t num_target_poses = opt_.number_target_poses();
    while (targetPoses_.size() > num_target_poses)
        targetPoses_.pop_front();


    if (!opt_.use_odom_twist())
    {
        lastTargetDir_ = estimateVel();

    }




}



void RobotController_Velocity_TT::odomCallback (const nav_msgs::OdometryConstPtr& odom)
{
    if (opt_.input_type() == VELTT_IT_ODOM)
    {
        geometry_msgs::PoseStamped tpose = Odom2Pose(*odom);
        processPose(tpose);
        if (opt_.use_odom_twist())
        {
            geometry_msgs::Vector3Stamped in_vec,out_vec;
            in_vec.header = odom->header;
            in_vec.vector = odom->twist.twist.linear;
            pose_tracker_->getTransformListener().transformVector(getFixedFrame(),in_vec,out_vec);
            lastTargetDir_ = cv::Vec2f(out_vec.vector.x, out_vec.vector.y);
        }
    }
}




