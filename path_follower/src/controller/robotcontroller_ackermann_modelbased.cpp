// HEADER
#include <path_follower/controller/robotcontroller_ackermann_modelbased.h>
// PROJECT
#include <path_follower/factory/controller_factory.h>


REGISTER_ROBOT_CONTROLLER(RobotController_Ackermann_ModelBased, ackermann_modelbased, ackermann);

RobotController_Ackermann_ModelBased::RobotController_Ackermann_ModelBased():
    RobotController_ModelBased()
{
}


void RobotController_Ackermann_ModelBased::initialize()
{
    RobotController_ModelBased::initialize();
    wheelBaseLength_ = config.wheelsConfig_.wheelPosRobotX*2.0;

}


double convert_trans_rot_vel_to_steering_angle(double v,double omega, double wheelbase)
{
  if (omega == 0 || v == 0) return 0;

  double radius = v / omega;
  return atan(wheelbase / radius);

}

void RobotController_Ackermann_ModelBased::publishMoveCommand(const MoveCommand &cmd) const
{
    geometry_msgs::Twist msg;
    msg.linear.x  = cmd.getVelocity();
    msg.linear.y  = 0;
    msg.angular.z = convert_trans_rot_vel_to_steering_angle(cmd.getVelocity(),cmd.getRotationalVelocity(),wheelBaseLength_);

    cmd_pub_.publish(msg);
}

