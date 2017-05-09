#ifndef CONTROLLER_FACTORY_H
#define CONTROLLER_FACTORY_H

#include <path_follower/utils/path_follower_config.h>
#include <path_follower/factory/abstract_factory.h>

#include <memory>
#include <functional>
#include <pluginlib/class_loader.h>


namespace ros {
class Duration;
}

class PathFollowerParameters;

class RobotController;
class LocalPlanner;
class CollisionAvoider;
class PathFollower;

class PoseTracker;

class ControllerFactory : public AbstractFactory
{
public:
    ControllerFactory(const PathFollowerParameters &opt);

public:
    std::shared_ptr<RobotController> makeController(const std::string &name);

    template <typename Controller>
    static void registerController(const std::string& type, const std::string& collision_detector)
    {
        controller_constructors_.emplace(toLower(type), [](){
            return std::make_shared<Controller>();
        });
        default_collision_detectors_[type] = collision_detector;
    }

    void loadAllControllers(std::vector<std::shared_ptr<RobotController>>& out);

    std::string getDefaultCollisionAvoider(const std::string& controller) const;


private:
    const PathFollowerParameters& opt_;

    pluginlib::ClassLoader<RobotController> controller_loader;
    static std::map<std::string, std::function<std::shared_ptr<RobotController>()>> controller_constructors_;

    static std::map<std::string, std::string> default_collision_detectors_;
};



template <typename Controller>
class ControllerFactoryRegistration
{
public:
    ControllerFactoryRegistration(const std::string& type, const std::string& collision_detector)
    {
        ControllerFactory::registerController<Controller>(type, collision_detector);
    }
};

#define REGISTER_ROBOT_CONTROLLER(class_t, type, collision_detector) \
ControllerFactoryRegistration<class_t> register_##type(#type, #collision_detector)

#endif // CONTROLLER_FACTORY_H
