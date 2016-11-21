/// HEADER
#include <path_follower/obstacle_avoidance/obstacleavoider.h>

ObstacleAvoider::ObstacleAvoider()
    : tf_listener_(nullptr)
{

}

void ObstacleAvoider::setTransformListener(const tf::TransformListener *tf_listener)
{
    tf_listener_ = tf_listener;
}

std::shared_ptr<ObstacleCloud const> ObstacleAvoider::getObstacles() const
{
    return obstacles_;
}

void ObstacleAvoider::setObstacles(std::shared_ptr<ObstacleCloud const> obstacles)
{
    obstacles_ = obstacles;
}
