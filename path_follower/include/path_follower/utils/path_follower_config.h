#ifndef PATH_FOLLOWER_CONFIG_H
#define PATH_FOLLOWER_CONFIG_H

#include <memory>

class RobotController;
class AbstractLocalPlanner;
class CollisionAvoider;

struct PathFollowerConfigName
{
    std::string controller;
    std::string local_planner;
    std::string collision_avoider;

    bool operator < (const PathFollowerConfigName& rhs) const
    {
        if(controller < rhs.controller) {
            return true;
        }
        if(local_planner < rhs.local_planner) {
            return true;
        }
        if(collision_avoider < rhs.collision_avoider) {
            return true;
        }

        return false;
    }
};

struct PathFollowerConfig
{
    //! The robot controller is responsible for everything that is dependend on robot model and controller type.
    std::shared_ptr<RobotController> controller_;
    std::shared_ptr<AbstractLocalPlanner> local_planner_;
    std::shared_ptr<CollisionAvoider> collision_avoider_;
};

#endif // PATH_FOLLOWER_CONFIG_H
