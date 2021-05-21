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

struct PathFollowerParameters;

class RobotController;
class AbstractLocalPlanner;
class CollisionAvoider;
class PathFollower;

class PoseTracker;

/**
 * @brief The ControllerFactory class is responsible for creating instance
 *        of the RobotController class.
 * @see RobotController
 */
class ControllerFactory : public AbstractFactory
{
public:
    /**
     * @brief ControllerFactory
     * @param opt Configuration to use when creating controllers
     */
    ControllerFactory(const PathFollowerParameters &opt);

public:
    /**
     * @brief makeController creates an instance of the RobotController identified by <name>.
     * @param name The name of the robot controller to instanciate
     * @return A shared pointer to the controller
     */
    std::shared_ptr<RobotController> makeController(const std::string &name);

    /**
     * @brief registerController registers the class <Controller> with the identifier <type>.
     * @param type Identifier for creating instances of type <Controller>
     * @param collision_detector the identifier of the CollisionDetector class to use
     */
    template <typename Controller>
    static void registerController(const std::string& type, const std::string& collision_detector)
    {
        controller_constructors_.emplace(toLower(type), [](){
            return std::make_shared<Controller>();
        });
        default_collision_detectors_[type] = collision_detector;
    }

    /**
     * @brief loadAllControllers Creates instances of all robot controllers
     * @param controllers A list containing an instance of each available robot controller
     */
    void loadAllControllers(std::vector<std::shared_ptr<RobotController>>& out);

    /**
     * @brief getDefaultCollisionAvoider accesses the default collision avoider identifiers.
     * @param controller Name of the controller for which the default collision avoider is requested.
     * @return Name of the default collision avoider for the robot controller <controller>
     */
    std::string getDefaultCollisionAvoider(const std::string& controller) const;


private:
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
