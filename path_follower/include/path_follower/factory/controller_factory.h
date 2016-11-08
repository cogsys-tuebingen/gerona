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

public:
    void construct(std::shared_ptr<RobotController>& out_controller,
                   std::shared_ptr<LocalPlanner>& out_local_planner,
                   std::shared_ptr<ObstacleAvoider>& out_obstacle_avoider);

private:
    std::shared_ptr<RobotController> makeController(const std::string &name);
    std::shared_ptr<LocalPlanner> makeLocalPlanner(const std::string &name);
    std::shared_ptr<ObstacleAvoider> makeObstacleAvoider(const std::string &name);

private:
    PathFollower &follower_;
    const PathFollowerParameters& opt_;
    PoseTracker& pose_tracker_;
};

#endif // CONTROLLER_FACTORY_H
