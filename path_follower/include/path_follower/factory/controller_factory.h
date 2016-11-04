#ifndef CONTROLLER_FACTORY_H
#define CONTROLLER_FACTORY_H

#include <memory>


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

    std::shared_ptr<RobotController> makeController(const std::string &name);

    std::shared_ptr<LocalPlanner> makeLocalPlanner(const std::string& name, RobotController &controller,
                                                   PoseTracker& pose_listener,
                                                   const ros::Duration& uinterval);

    std::shared_ptr<ObstacleAvoider> makeObstacleAvoider(const std::string& name,
                                                         PoseTracker &pose_listener,
                                                         std::shared_ptr<RobotController>& controller);

private:
    PathFollower &follower_;
    const PathFollowerParameters& opt_;
};

#endif // CONTROLLER_FACTORY_H
