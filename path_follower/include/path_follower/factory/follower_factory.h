#ifndef FOLLOWER_FACTORY_H
#define FOLLOWER_FACTORY_H

#include <path_follower/factory/abstract_factory.h>
#include <path_follower/utils/path_follower_config.h>

#include <memory>
#include <functional>
#include <vector>

class ControllerFactory;
class LocalPlannerFactory;
class CollisionAvoiderFactory;

class PathFollower;
class PathFollowerParameters;
class PoseTracker;

/**
 * @brief The FollowerFactory class is responsible for creating instances of all
 *        classes necessary for path following.
 * @see CollisionAvoider
 */
class FollowerFactory : public AbstractFactory
{
public:
    /**
     * @brief FollowerFactory
     * @param follower the instance of path follower
     */
    FollowerFactory(PathFollower &follower);

    /**
     * @brief construct creates instances of all necessary classes needed for following a path.
     * @param config The names of the requested classes of RobotController, LocalPlanner and CollisionAvoider
     * @return A shared pointer to a collection of all constructed objects
     */
    std::shared_ptr<PathFollowerConfig> construct(const PathFollowerConfigName &config);

    /**
     * @brief loadAll Creates instances of all robot controllers
     * @param controllers A list containing an instance of each available robot controller
     */
    void loadAll(std::vector<std::shared_ptr<RobotController> > &controllers);

private:
    const PathFollowerParameters& opt_;

    PathFollower &follower_;
    PoseTracker& pose_tracker_;

    std::shared_ptr<ControllerFactory> controller_factory_;
    std::shared_ptr<LocalPlannerFactory> local_planner_factory_;
    std::shared_ptr<CollisionAvoiderFactory> collision_avoider_factory_;
};

#endif // FOLLOWER_FACTORY_H
