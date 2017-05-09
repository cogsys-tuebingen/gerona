/// HEADER
#include <path_follower/factory/collision_avoider_factory.h>

#include <path_follower/collision_avoidance/none_avoider.hpp>
#include <path_follower/collision_avoidance/collision_detector_ackermann.h>
#include <path_follower/collision_avoidance/collision_detector_omnidrive.h>

std::shared_ptr<CollisionAvoider> CollisionAvoiderFactory::makeObstacleAvoider(const std::string &name)
{
    if(name == "default_collision_avoider") {
        return std::make_shared<CollisionDetectorOmnidrive>();
    } else {
        if (name == "omnidive") {
            return std::make_shared<CollisionDetectorOmnidrive>();
        } else if (name == "ackermann") {
            return std::make_shared<CollisionDetectorAckermann>();
        }
    }
    ROS_WARN_STREAM("No collision_avoider defined with the name '" << name << "'. Defaulting to Omnidrive.");
    return std::make_shared<CollisionDetectorOmnidrive>();
}
