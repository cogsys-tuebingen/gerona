#ifndef PATH_FOLLOWER_CONFIG_H
#define PATH_FOLLOWER_CONFIG_H

#include <memory>

class RobotController;
class LocalPlanner;
class ObstacleAvoider;

struct PathFollowerConfig
{
    //! The robot controller is responsible for everything that is dependend on robot model and controller type.
    std::shared_ptr<RobotController> controller_;
    std::shared_ptr<LocalPlanner> local_planner_;
    std::shared_ptr<ObstacleAvoider> obstacle_avoider_;
};

#endif // PATH_FOLLOWER_CONFIG_H
