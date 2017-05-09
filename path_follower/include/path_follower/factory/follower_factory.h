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

class FollowerFactory : public AbstractFactory
{
public:
    FollowerFactory(PathFollower &follower);

    std::shared_ptr<PathFollowerConfig> construct(const PathFollowerConfigName &config);

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
