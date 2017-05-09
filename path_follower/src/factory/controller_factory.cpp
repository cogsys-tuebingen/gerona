/// HEADER
#include <path_follower/factory/controller_factory.h>

#include <path_follower/pathfollower.h>

// Controller/Models
#include <path_follower/controller/robotcontroller.h>

#include <path_follower/utils/pose_tracker.h>

// SYSTEM
#include <stdexcept>

std::map<std::string, std::function<std::shared_ptr<RobotController>()>> ControllerFactory::controller_constructors_;
std::map<std::string, std::string> ControllerFactory::default_collision_detectors_;

ControllerFactory::ControllerFactory(const PathFollowerParameters &opt)
    : opt_(opt),

      controller_loader("path_follower", "RobotController")
{

}


std::shared_ptr<RobotController> ControllerFactory::makeController(const std::string& name)
{
    std::string name_low = toLower(name);

    auto pos = controller_constructors_.find(name_low);
    if(pos != controller_constructors_.end()) {
        return pos->second();
    }

    std::vector<std::string> classes = controller_loader.getDeclaredClasses();
    std::vector<std::string> classes_low;
    classes_low.reserve(classes.size());
    for(const std::string& s : classes) {
        classes_low.push_back(toLower(s));
    }

    std::string plugin_class;

    for(int i = 0, n = classes.size(); i < n; ++i) {
        const std::string& available = classes_low.at(i);

        // first try direct name
        if(available == name_low) {
            plugin_class = classes.at(i);
            break;
        }

        std::string conventional = std::string("robotcontroller_") + name_low;

        // then try the conventional naming scheme
        if(conventional == available) {
            plugin_class = classes.at(i);
            break;
        }
    }

    if(!plugin_class.empty() && controller_loader.isClassAvailable(plugin_class)) {
        ROS_INFO("Loading robot controller plugin '%s'", plugin_class.c_str());

        controller_constructors_[name_low] = [this, plugin_class]() -> std::shared_ptr<RobotController> {
            return std::shared_ptr<RobotController>(controller_loader.createUnmanagedInstance(plugin_class));
        };

        return controller_constructors_[name_low]();
    }

    throw std::logic_error(std::string("Unknown robot controller: ") + name + ". Shutdown.");
}


void ControllerFactory::loadAllControllers(std::vector<std::shared_ptr<RobotController>>& out)
{
    for(auto pair: controller_constructors_) {
        auto constructor = pair.second;
        out.push_back(constructor());
    }
}

std::string ControllerFactory::getDefaultCollisionAvoider(const std::string &controller) const
{
    auto key = default_collision_detectors_.find(controller);
    if(key != default_collision_detectors_.end()) {
        return key->second;
    }

    return {};
}
