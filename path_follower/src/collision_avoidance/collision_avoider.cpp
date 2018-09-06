/// HEADER
#include <path_follower/collision_avoidance/collision_avoider.h>

/// PROJECT
#include <path_follower/utils/obstacle_cloud.h>

CollisionAvoider::CollisionAvoider()
    : tf_listener_(nullptr),
      robot_frame_("base_link")
{

}

void CollisionAvoider::setTransformListener(const tf::TransformListener *tf_listener)
{
    tf_listener_ = tf_listener;
}

void CollisionAvoider::setRobotFrameId(const std::string& frame_id)
{
    robot_frame_ = frame_id;
}

bool CollisionAvoider::hasObstacles() const
{
    return obstacles_ != nullptr && !obstacles_->empty();
}

std::shared_ptr<ObstacleCloud const> CollisionAvoider::getObstacles() const
{
    if(!obstacles_) {
        ROS_FATAL("tried to access a non-existent obstacle cloud");
        std::abort();
    }
    return obstacles_;
}

void CollisionAvoider::setObstacles(std::shared_ptr<ObstacleCloud const> obstacles)
{
    obstacles_ = obstacles;
}

void CollisionAvoider::setExternalError(int externalError)
{
    externalError_ = externalError;
}
