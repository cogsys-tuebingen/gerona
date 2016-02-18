#include <path_follower/utils/movecommand.h>

MoveCommand::MoveCommand(bool can_rotate, bool torque_mode):
    move_dir_(1,0),
    velocity_(0),
    rot_velocity_(0),
    use_rotation_(can_rotate),
    use_torque_(torque_mode)
{
}

bool MoveCommand::isValid() const
{
    return isValid(move_dir_[0])
            && isValid(velocity_)
            && isValid(move_dir_[1])
            && isValid(rot_velocity_)
            && isValid(fl_torque_)
            && isValid(fr_torque_)
            && isValid(br_torque_)
            && isValid(bl_torque_);
}

Eigen::Vector2f MoveCommand::getDirection() const
{
    return move_dir_;
}

Eigen::Vector2f MoveCommand::getVelocityVector() const
{
    return velocity_ * move_dir_;
}

float MoveCommand::getDirectionAngle() const
{
    return atan2(move_dir_[1], move_dir_[0]);
}

float MoveCommand::getVelocity() const
{
    return velocity_;
}

float MoveCommand::getRotationalVelocity() const
{
    assert(canRotate());
    return rot_velocity_;
}

double MoveCommand::getWheelTorqueFL() const
{
    return fl_torque_;
}

double MoveCommand::getWheelTorqueFR() const
{
    return fr_torque_;
}

double MoveCommand::getWheelTorqueBR() const
{
    return br_torque_;
}

double MoveCommand::getWheelTorqueBL() const
{
    return bl_torque_;
}

bool MoveCommand::canRotate() const
{
    return use_rotation_;
}

bool MoveCommand::useTorque() const
{
    return use_torque_;
}

void MoveCommand::setDirection(const Eigen::Vector2f &dir)
{
    move_dir_ = dir.normalized();
}

void MoveCommand::setDirection(float angle)
{
    move_dir_[0] = cos(angle);
    move_dir_[1] = sin(angle);
}

void MoveCommand::setVelocity(float v)
{
    velocity_ = v;
}

void MoveCommand::setRotationalVelocity(float omega)
{
    assert(canRotate());
    rot_velocity_ = omega;
}

void MoveCommand::setWheelTorques(double fl, double fr, double br, double bl){
    assert(useTorque());
    fl_torque_ = fl;
    fr_torque_ = fr;
    br_torque_ = br;
    bl_torque_ = bl;
}

bool MoveCommand::isValid(float val) const
{
    return !isnan(val) && val != -INFINITY && val != INFINITY;
}
