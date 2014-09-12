#include "coursepredictor.h"
#include "pathfollower.h"
#include "behaviours.h"

#include <utils_general/MathHelper.h>
#include <tf/tf.h>

using namespace std;
using namespace Eigen;

CoursePredictor::CoursePredictor(PathFollower *path_driver):
    path_driver_(path_driver),
    update_intervall_(0.1),
    last_update_time_(0),
    last_positions_(5) //TODO: parameter for buffer size?
{
}

void CoursePredictor::update()
{
    if (ros::Time::now() - last_update_time_ < update_intervall_) {
        return;
    }

    last_positions_.push_back(path_driver_->getRobotPose().head<2>());
    last_update_time_ = ros::Time::now();
}

void CoursePredictor::reset()
{
    last_update_time_   = ros::Time(0);
    last_positions_.clear();
}

Eigen::Vector2d CoursePredictor::predictDirectionOfMovement()
{
    //TODO: more sophisticated prediction

    Vector2d direction(0, 0);

    if (!last_update_time_.isZero()) {
        // transform last position to robot frame
        geometry_msgs::PoseStamped last_pos_msg;
        last_pos_msg.pose.position.x = last_positions_.back().x();
        last_pos_msg.pose.position.y = last_positions_.back().y();
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
    if (last_positions_.size() < 2) {
        return Vector2d(0,0);
    }

    // convert circular buffer to vector
    vector<Vector2d> points;
    points.assign(last_positions_.begin(), last_positions_.end());
    // at current position
    points.push_back(path_driver_->getRobotPose().head<2>());

    MathHelper::Line line = MathHelper::FitLinear(points);

    /// make sure, the vector is pointing forward!
    // rough direction estimation: vector from oldest to newest point in the buffer (new points are pushed to the back)
    Vector2d rough_dir = last_positions_.back() - last_positions_.front();
    double angle = MathHelper::Angle(line.direction, rough_dir);
    if (angle > M_PI/2) {
        line.direction *= -1;
    }

    /** transform direction vector to robot frame **/
    Eigen::Vector2d direction;
    {
        geometry_msgs::PoseStamped line_direction_as_pose_msg;
        line_direction_as_pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(MathHelper::Angle( line.direction ));

        geometry_msgs::PoseStamped local_msg;
        if ( !path_driver_->transformToLocal(line_direction_as_pose_msg, local_msg) ) {
            //FIXME: set status
            //path_driver_->getActiveBehaviour()->setStatus(path_msgs::FollowPathResult::MOTION_STATUS_SLAM_FAIL);
            throw new BehaviourEmergencyBreak(*path_driver_);
        }

        tf::Quaternion rot;
        tf::quaternionMsgToTF(local_msg.pose.orientation, rot);
        tf::Vector3 direction_tf = tf::quatRotate(rot, tf::Vector3(1,0,0));
        direction = Eigen::Vector2d(direction_tf.x(), direction_tf.y());
    }
    /** transformation end **/

    return direction;
}

ros::Duration CoursePredictor::getUpdateIntervall() const
{
    return update_intervall_;
}

void CoursePredictor::setUpdateIntervall(const ros::Duration &update_intervall)
{
    update_intervall_ = update_intervall;
}

