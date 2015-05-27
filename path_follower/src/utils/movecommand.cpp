#include <path_follower/utils/movecommand.h>

MoveCommand::MoveCommand(bool can_rotate):
    move_dir_(1,0),
    velocity_(0),
    rot_velocity_(0),
    use_rotation_(can_rotate)
{
}

bool MoveCommand::isValid() const
{
    return isValid(move_dir_[0])
            && isValid(velocity_)
            && isValid(move_dir_[1])
            && isValid(rot_velocity_);
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

bool MoveCommand::canRotate() const
{
    return use_rotation_;
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

bool MoveCommand::isValid(float val) const
{
    return !isnan(val) && val != -INFINITY && val != INFINITY;
}
