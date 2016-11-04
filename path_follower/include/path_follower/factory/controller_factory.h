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
    PathFollower &follower_;
    const PathFollowerParameters& opt_;
    PoseTracker& pose_tracker_;
};

#endif // CONTROLLER_FACTORY_H
