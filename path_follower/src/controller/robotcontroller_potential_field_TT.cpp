// HEADER
#include <path_follower/controller/robotcontroller_potential_field_TT.h>

// THIRD PARTY
#include <tf/tf.h>
#include <nav_msgs/Path.h>

// PROJECT
#include <path_follower/utils/pose_tracker.h>
#include <path_follower/utils/obstacle_cloud.h>
#include <interpolation.h>
#include <path_follower/pathfollowerparameters.h>
#include <path_follower/obstacle_avoidance/obstacleavoider.h>
#include <cslibs_utils/MathHelper.h>

// SYSTEM
#include <boost/algorithm/clamp.hpp>


RobotController_Potential_Field_TT::RobotController_Potential_Field_TT():
    RobotController_Potential_Field()
{
}

void RobotController_Potential_Field_TT::setGoalPosition()
{

    Eigen::Vector3d current_pose = pose_tracker_->getRobotPose();
    double x_meas = current_pose[0];
    double y_meas = current_pose[1];
    double theta_meas = current_pose[2];


    ///PERSON FOLLOWING
    double s_diff = std::numeric_limits<double>::max();
    // desired distance behind the person in path coordinates
    double s_dist = 2.0;
    tf::Point goal;
    for(int i = proj_ind_; i < path_interpol.n()-1 ; i++){
        if(fabs(fabs(path_interpol.s(path_interpol.n()-1) - path_interpol.s(i)) - s_dist) < s_diff){
            s_diff = fabs(fabs(path_interpol.s(path_interpol.n()-1) - path_interpol.s(i)) - s_dist);
            tf::Point goal_tmp(path_interpol.p(i), path_interpol.q(i), 0.0);
            goal = goal_tmp;
        }
        if(fabs(fabs(path_interpol.s(path_interpol.n()-1) - path_interpol.s(proj_ind_)) - s_dist) < 1e-1){
            vn_ = 0.0;
        }
    }


    mGoalPosX = goal.getX();
    mGoalPosY = goal.getY();

    double dx = mGoalPosX - x_meas;
    double dy = mGoalPosY - y_meas;

    FAttX = opt_.kAtt()*(dx*cos(-theta_meas) - dy*sin(-theta_meas));
    FAttY = opt_.kAtt()*(dx*sin(-theta_meas) + dy*cos(-theta_meas));

}


