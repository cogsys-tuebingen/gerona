#ifndef LOCAL_PLANNER_FACTORY_H
#define LOCAL_PLANNER_FACTORY_H

#include <path_follower/parameters/local_planner_parameters.h>
#include <path_follower/factory/abstract_factory.h>

#include <memory>
#include <functional>
#include <pluginlib/class_loader.h>

class LocalPlanner;

class LocalPlannerFactory : public AbstractFactory
{
public:
    LocalPlannerFactory(const LocalPlannerParameters& opt);

    std::shared_ptr<LocalPlanner> makeConstrainedLocalPlanner(const std::string &name);

    template <typename Planner>
    static void registerPlanner(const std::string& type)
    {
        planner_constructors_.emplace(toLower(type), [](){
            return std::make_shared<Planner>();
        });
    }

private:
    std::shared_ptr<LocalPlanner> makeLocalPlanner(const std::string &name);

private:
    const LocalPlannerParameters& opt_;

    pluginlib::ClassLoader<LocalPlanner> planner_loader_;
    static std::map<std::string, std::function<std::shared_ptr<LocalPlanner>()>> planner_constructors_;
};




template <typename Planner>
class LocalPlannerFactoryRegistration
{
public:
    LocalPlannerFactoryRegistration(const std::string& type)
    {
        LocalPlannerFactory::registerPlanner<Planner>(type);
    }
};

#define REGISTER_LOCAL_PLANNER(class_t, type) \
LocalPlannerFactoryRegistration<class_t> register_##type(#type)
#endif // LOCAL_PLANNER_FACTORY_H
