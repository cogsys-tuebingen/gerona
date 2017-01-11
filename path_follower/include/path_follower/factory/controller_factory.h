#ifndef CONTROLLER_FACTORY_H
#define CONTROLLER_FACTORY_H

#include <path_follower/utils/path_follower_config.h>

#include <memory>
#include <pluginlib/class_loader.h>


namespace ros {
class Duration;
}

class PathFollowerParameters;

class RobotController;
class LocalPlanner;
class ObstacleAvoider;
class PathFollower;

class PoseTracker;

class ControllerFactory
{
public:
    ControllerFactory(PathFollower &follower);

public:
    std::shared_ptr<PathFollowerConfig> construct(const PathFollowerConfigName &config);

private:
    std::shared_ptr<RobotController> makeController(const std::string &name);
    std::shared_ptr<ObstacleAvoider> makeObstacleAvoider(const std::string &name);

    std::shared_ptr<LocalPlanner> makeConstrainedLocalPlanner(const std::string &name);
    std::shared_ptr<LocalPlanner> makeLocalPlanner(const std::string &name);

private:
    PathFollower &follower_;
    const PathFollowerParameters& opt_;
    PoseTracker& pose_tracker_;

    pluginlib::ClassLoader<RobotController> controller_loader;

};

#endif // CONTROLLER_FACTORY_H
