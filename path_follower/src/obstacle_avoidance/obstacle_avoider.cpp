/// HEADER
#include <path_follower/obstacle_avoidance/obstacleavoider.h>

/// PROJECT
#include <path_follower/utils/obstacle_cloud.h>

ObstacleAvoider::ObstacleAvoider()
    : tf_listener_(nullptr)
{

}

void ObstacleAvoider::setTransformListener(const tf::TransformListener *tf_listener)
{
    tf_listener_ = tf_listener;
}

bool ObstacleAvoider::hasObstacles() const
{
    return obstacles_ != nullptr && !obstacles_->empty();
}

std::shared_ptr<ObstacleCloud const> ObstacleAvoider::getObstacles() const
{
    if(!obstacles_) {
        ROS_FATAL("tried to access a non-existent obstacle cloud");
        std::abort();
    }
    return obstacles_;
}

void ObstacleAvoider::setObstacles(std::shared_ptr<ObstacleCloud const> obstacles)
{
    obstacles_ = obstacles;
}
