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
    RobotController(),
    cmd_(this)
{



}

void RobotController_Velocity_TT::stopMotion()
{

    cmd_.speed = 0;
    cmd_.direction_angle = 0;
    cmd_.rotation = 0;

    MoveCommand mcmd = cmd_;
    publishMoveCommand(mcmd);
}

void RobotController_Velocity_TT::initialize()
{
    RobotController::initialize();

    //reset the index of the current point on the path



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

}

/*
double RobotController_EKM::computeSpeed()
{

}
*/
cv::Vec2f RobotController_Velocity_TT::CalcForceRep(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    float min_dist = opt_.dist_thresh();
    float curDist = 0;
    float t1,krep = opt_.krep();
    float min_dist_inv = 1.0f/min_dist;
    cv::Vec2f t2,tfrep;
    cv::Vec2f curP(0,0);
    cv::Vec2f sumF(0,0);
    int counter = 0;


    for(const pcl::PointXYZ& pt : cloud) {
        curP[0] = pt.x;
        curP[1] = pt.y;

        curDist = std::sqrt(curP.dot(curP));
        if(curDist < min_dist){

            t1 = (1.0f/curDist - min_dist_inv);
            t2 = (1.0f/(curDist*curDist))* ((-curP)*min_dist_inv);
            tfrep = krep*t1*t2;
            sumF = sumF+tfrep;
            counter++;
        }

    }
    cv::Vec2f result = sumF * (1.0f/(float)counter);
    return result;
}

cv::Vec2f RobotController_Velocity_TT::CalcForceRep()
{
    auto obstacle_cloud = collision_avoider_->getObstacles();
    const pcl::PointCloud<pcl::PointXYZ>& cloud = *obstacle_cloud->cloud;

    if(cloud.header.frame_id == PathFollowerParameters::getInstance()->robot_frame()) {
        return CalcForceRep(cloud);
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
        return CalcForceRep(tcloud);
    }

}



RobotController::MoveCommandStatus RobotController_Velocity_TT::computeMoveCommand(MoveCommand *cmd)
{
    // omni drive can rotate.
    *cmd = MoveCommand(true);

    if(path_interpol.n() < 2) {
        ROS_ERROR("[Line] path is too short (N = %d)", (int) path_interpol.n());

        stopMotion();
        return MoveCommandStatus::REACHED_GOAL;
    }

    return MoveCommandStatus::OKAY;


}

void RobotController_Velocity_TT::publishMoveCommand(const MoveCommand &cmd) const
{
    geometry_msgs::Twist msg;
    msg.linear.x  = cmd.getVelocity();
    msg.linear.y  = 0;
    msg.angular.z = cmd.getRotationalVelocity();

    cmd_pub_.publish(msg);
}

