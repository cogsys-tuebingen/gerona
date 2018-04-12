// HEADER
#include <path_follower/controller/robotcontroller_ekm.h>

// THIRD PARTY


// PROJECT
#include <path_follower/factory/controller_factory.h>
#include <path_follower/utils/pose_tracker.h>


// SYSTEM

REGISTER_ROBOT_CONTROLLER(RobotController_EKM, ekm, default_collision_avoider);

//using namespace Eigen;


RobotController_EKM::RobotController_EKM():
    RobotController(),
    cmd_(this)
{
    prev_phi_d_ = 0;


}

void RobotController_EKM::stopMotion()
{

    cmd_.speed = 0;
    cmd_.direction_angle = 0;
    cmd_.rotation = 0;

    MoveCommand mcmd = cmd_;
    publishMoveCommand(mcmd);
}

void RobotController_EKM::initialize()
{
    RobotController::initialize();

    //reset the index of the current point on the path


    prev_phi_d_ = 0;
    has_prev_phi_d_ = false;
    //phi_ = -M_PI/2.0;
}



void RobotController_EKM::start()
{
    RobotController::start();
}

void RobotController_EKM::reset()
{
    RobotController::reset();

}

void RobotController_EKM::setPath(Path::Ptr path)
{
    RobotController::setPath(path);
}

/*
double RobotController_EKM::computeSpeed()
{

}
*/

void RobotController_EKM::DerivePathInterp(double fact)
{
    phi_p_.clear();
    for (int i = 0; i < path_interpol.n()-1;++i)
    {
        double x_p = (path_interpol.p(i+1)-path_interpol.p(i))*fact;
        double y_p = (path_interpol.q(i+1)-path_interpol.q(i))*fact;
        phi_p_.push_back(atan2(y_p,x_p));

    }
}

double GetAngleDifference(double a, double b)
{
    if (a > 3.0 && b < -3.0)
    {
        return (b+M_PI*2.0)-a;
    }
    if (a < -3.0 && b > 3.0)
    {
        return (a+M_PI*2.0)-b;
    }

    return a-b;
}

RobotController::MoveCommandStatus RobotController_EKM::computeMoveCommand(MoveCommand *cmd)
{
    // omni drive can rotate.
    *cmd = MoveCommand(true);

    if(path_interpol.n() < 2) {
        ROS_ERROR("[Line] path is too short (N = %d)", (int) path_interpol.n());

        stopMotion();
        return MoveCommandStatus::REACHED_GOAL;
    }

    /// get the pose as pose(0) = x, pose(1) = y, pose(2) = theta
    Eigen::Vector3d current_pose = pose_tracker_->getRobotPose();


    RobotController::findOrthogonalProjection();

    if(RobotController::isGoalReached(cmd)){
       return RobotController::MoveCommandStatus::REACHED_GOAL;
    }

    int k_p = proj_ind_+opt_.n();

    DerivePathInterp(1.0/opt_.ts());

    double xpr = current_pose(0);
    double ypr = current_pose(1);
    double phi = current_pose(2);

    double x_d = path_interpol.p(k_p);
    double y_d = path_interpol.q(k_p);

    double xp_d = opt_.u_d()*cos(phi_p_[k_p]);
    double yp_d = opt_.u_d()*sin(phi_p_[k_p]);

    double x_t = x_d - xpr;
    double y_t = y_d - ypr;

    double nu_x = xp_d + opt_.lx()*tanh(opt_.kx()*x_t/opt_.lx());
    double nu_y = yp_d + opt_.ly()*tanh(opt_.ky()*y_t/opt_.ly());


    double phi_d = atan2(nu_y,nu_x);

    if (!has_prev_phi_d_)
    {
        prev_phi_d_ = phi_d;
        has_prev_phi_d_ = true;
    }
    double phip_d = GetAngleDifference(phi_d,prev_phi_d_)/opt_.ts();

    prev_phi_d_ = phi_d;

    double phi_t = GetAngleDifference(phi_d , phi);

    double nu_p  = phip_d+ opt_.lw()*tanh(opt_.kw()*phi_t/opt_.lw());


    double u = (nu_x*cos(phi)+ nu_y*sin(phi)+ nu_p*opt_.a()*sin(opt_.alpha()));
    double w = (nu_p);


    cmd_.speed = u;
    cmd_.rotation = w;
    cmd_.direction_angle = 0;
    *cmd = cmd_;
    return MoveCommandStatus::OKAY;
/*
    for (unsigned int i = old_ind; i < path_interpol.n(); i++){

        double s_diff_curr = std::abs(path_interpol.s_new() - path_interpol.s(i));

        if(s_diff_curr < s_diff){

            s_diff = s_diff_curr;
            ind_ = i;

        }

    }

    if(old_ind != ind_) {
        path_->fireNextWaypointCallback();
    }
*/
    ///***///


    ///***///


}

void RobotController_EKM::publishMoveCommand(const MoveCommand &cmd) const
{
    geometry_msgs::Twist msg;
    msg.linear.x  = cmd.getVelocity();
    msg.linear.y  = 0;
    msg.angular.z = cmd.getRotationalVelocity();

    cmd_pub_.publish(msg);
}

