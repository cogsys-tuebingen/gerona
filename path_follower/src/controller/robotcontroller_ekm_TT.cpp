// HEADER
#include <path_follower/controller/robotcontroller_ekm_TT.h>

// THIRD PARTY


// PROJECT
#include <path_follower/factory/controller_factory.h>
#include <path_follower/utils/pose_tracker.h>
#include <path_follower/parameters/path_follower_parameters.h>


// SYSTEM
#include <cmath>


REGISTER_ROBOT_CONTROLLER(RobotController_EKM_TT, ekm_TT, default_collision_avoider);

//using namespace Eigen;


RobotController_EKM_TT::RobotController_EKM_TT():
    RobotController(),
    cmd_(this)
{
    prev_phi_d_ = 0;
    target_.x = 0;
    target_.y = 0;
    target_.orientation = 0;


}

void RobotController_EKM_TT::stopMotion()
{

    cmd_.speed = 0;
    cmd_.direction_angle = 0;
    cmd_.rotation = 0;

    MoveCommand mcmd = cmd_;
    publishMoveCommand(mcmd);
}

void RobotController_EKM_TT::initialize()
{
    RobotController::initialize();

    //reset the index of the current point on the path


    prev_phi_d_ = 0;
    has_prev_phi_d_ = false;
    //phi_ = -M_PI/2.0;
}



void RobotController_EKM_TT::start()
{
    RobotController::start();
}

void RobotController_EKM_TT::reset()
{
    RobotController::reset();

}



bool RobotController_EKM_TT::targetTransform2base(ros::Time& now,Waypoint pathEnd)
{
    tf::StampedTransform now_map_to_base;
    tf::Transformer* transformer_ = &pose_tracker_->getTransformListener();


    std::string world_frame = path_->getFrameId();
    std::string robot_frame = PathFollowerParameters::getInstance()->robot_frame();

    try{//Try to get the latest avaiable Transform
        transformer_->lookupTransform(world_frame, robot_frame, ros::Time(0), now_map_to_base);
    }catch(const tf::TransformException& ex){//if not available, then wait
        (void) ex;
        if(!transformer_->waitForTransform(world_frame, robot_frame, now, ros::Duration(0.1))){
            ROS_WARN_THROTTLE_NAMED(1, "local_path", "cannot transform map to odom");
            return false;
        }
        transformer_->lookupTransform(world_frame, robot_frame, now, now_map_to_base);
    }

    tf::Transform transform_correction = now_map_to_base.inverse();

    tf::Point pt(pathEnd.x, pathEnd.y, 0);
    pt = transform_correction * pt;
    target_.x = pt.x();
    target_.y = pt.y();

    tf::Quaternion rot = tf::createQuaternionFromYaw(pathEnd.orientation);
    rot = transform_correction * rot;
    target_.orientation = tf::getYaw(rot);

    return true;
}



void RobotController_EKM_TT::setPath(Path::Ptr path)
{
    RobotController::setPath(path);
    Waypoint pathEnd;
    path_interpol.get_end(pathEnd);
    ros::Time now =  ros::Time::now();
    targetTransform2base(now, pathEnd);
    //DerivePathInterp(1.0/opt_.ts());

}


RobotController::MoveCommandStatus RobotController_EKM_TT::computeMoveCommand(MoveCommand *cmd)
{
    // omni drive can rotate.
    *cmd = MoveCommand(true);

    if(path_interpol.n() < 2) {
        ROS_ERROR("[Line] path is too short (N = %d)", (int) path_interpol.n());

        stopMotion();
        return MoveCommandStatus::REACHED_GOAL;
    }


    double tY = target_.y;
    double tX = target_.x;

    //z_hum_pos = tx
    //x_hum_pos = ty

    double phi_t = -atan2(tY,tX);

    double x_t = tX - 1.8*cos(phi_t);

    double kw = 1.5 + 50*(phi_t*phi_t);
    double kx = 0.1 + 0.5*(x_t*x_t);
    double lw = 1;
    double lx = 2;

    double w_cmd = lw*tanh(kw*phi_t/lw);

    double u_cmd = lx*tanh(kx*x_t/lx);


    /*
    double maxW = 0.3;

    double w_cmd = std::min(atan2(tY,tX),maxW);

    double maxU = 0.5;
    double dist = sqrt(tX*tX+tY*tY);

    double u_cmd = std::min(dist,maxU);
*/



    if (std::abs(w_cmd)<0.001) w_cmd = 0;

    if (std::abs(u_cmd)<0.001) u_cmd = 0;

    //ROS_ERROR_STREAM_THROTTLE(4.0,"EKM Debug: tx: " << tX << " tY: " << tY);
    ROS_ERROR_STREAM("EKM Debug: tx: " << tX << " tY: " << tY << " phi: " << phi_t << " robot: " << PathFollowerParameters::getInstance()->robot_frame() << " path: " << path_->getFrameId());


    cmd_.speed = u_cmd;
    cmd_.rotation = w_cmd;
    cmd_.direction_angle = 0;
    *cmd = cmd_;
    return MoveCommandStatus::OKAY;


}

void RobotController_EKM_TT::publishMoveCommand(const MoveCommand &cmd) const
{
    geometry_msgs::Twist msg;
    msg.linear.x  = cmd.getVelocity();
    msg.linear.y  = 0;
    msg.angular.z = cmd.getRotationalVelocity();

    cmd_pub_.publish(msg);
}

