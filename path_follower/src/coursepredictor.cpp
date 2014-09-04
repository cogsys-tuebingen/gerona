#include "coursepredictor.h"
#include "pathfollower.h"
#include "behaviours.h"

using namespace std;
using namespace Eigen;

CoursePredictor::CoursePredictor(PathFollower *path_driver):
    path_driver_(path_driver),
    update_intervall_(0.3),
    last_update_time_(0),
    last_position_(0,0),
    smoothed_direction_(0,0)
{
}

void CoursePredictor::update()
{
    if (ros::Time::now() - last_update_time_ < update_intervall_) {
        return;
    }

    last_position_ = path_driver_->getRobotPose().head<2>();
    last_update_time_ = ros::Time::now();
}

void CoursePredictor::reset()
{
    last_update_time_   = ros::Time(0);
    last_position_      = Vector2d(0,0);
    smoothed_direction_ = Vector2d(0,0);
}

Eigen::Vector2d CoursePredictor::predictDirectionOfMovement()
{
    //TODO: more sophisticated prediction

    Vector2d direction(0, 0);

    if (!last_update_time_.isZero()) {
        // transform last position to robot frame
        geometry_msgs::PoseStamped last_pos_msg;
        last_pos_msg.pose.position.x = last_position_.x();
        last_pos_msg.pose.position.y = last_position_.y();
        last_pos_msg.pose.orientation.w = 1;

        Vector3d last_position;
        if ( !path_driver_->transformToLocal(last_pos_msg, last_position) ) {
            //FIXME: set status
            //path_driver_->getActiveBehaviour()->setStatus(path_msgs::FollowPathResult::MOTION_STATUS_SLAM_FAIL);
            throw new BehaviourEmergencyBreak(*path_driver_);
        }

        // calculate direction of movement (current_pos - last_pos, where current_pos = 0)
        direction = -last_position.head<2>();
    }

    return direction;
}

Eigen::Vector2d CoursePredictor::smoothedDirection()
{
    //FIXME: I'm not so happy with this, it is rather a dirty hack to make obstacle detection stable even when the robot
    //      makes slight sideways movements. I think, there must be a better, more reliable solution...

    Vector2d current_pos = path_driver_->getRobotPose().head<2>();

    if (last_update_time_.isZero()) { // only true in initial call
        last_position_smoothed_ = current_pos;
    }


    // Only update, if the robot has moved at least a certain distance.
    double driven_dist = (last_position_smoothed_ - current_pos).norm();

    //ROS_DEBUG("PSDOM: driven_dist = %g", driven_dist);
    if (driven_dist > 0.05) {
        // update
        Vector2d direction = predictDirectionOfMovement();

        const float r = 0.0; //FIXME
        smoothed_direction_ = r*smoothed_direction_ + (1-r)*direction;

        last_position_smoothed_ = current_pos;

        //ROS_DEBUG_STREAM("PSDOM: smoothed_dir: " << smoothed_direction_);
    }

    //ROS_DEBUG("PSDOM: angle = %g", atan2(smoothed_direction_[1], smoothed_direction_[0]));
    //return atan2(smoothed_direction_[1], smoothed_direction_[0]);
    return smoothed_direction_;
}

ros::Duration CoursePredictor::getUpdateIntervall() const
{
    return update_intervall_;
}

void CoursePredictor::setUpdateIntervall(const ros::Duration &update_intervall)
{
    update_intervall_ = update_intervall;
}

