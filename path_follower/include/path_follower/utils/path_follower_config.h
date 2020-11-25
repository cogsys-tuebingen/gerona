#ifndef PATH_FOLLOWER_CONFIG_H
#define PATH_FOLLOWER_CONFIG_H

#include <memory>
#include <string>

class RobotController;
class AbstractLocalPlanner;
class CollisionAvoider;

/**
 * @brief The PathFollowerConfigName struct represents a configuration for
 *        following a path. This configuration is used by the factories to create
 *        instances of the nescessary classes in the form of PathFollowerConfig.
 * @see PathFollowerConfig
 */
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

/**
 * @brief The PathFollowerConfig struct holds instances of the necessary classes
 *        for path following and represents a single configuration.
 */
struct PathFollowerConfig
{
    //! The robot controller is responsible for everything that is dependend on robot model and controller type.
    std::shared_ptr<RobotController> controller_;
    std::shared_ptr<AbstractLocalPlanner> local_planner_;
    std::shared_ptr<CollisionAvoider> collision_avoider_;
};

#endif // PATH_FOLLOWER_CONFIG_H
