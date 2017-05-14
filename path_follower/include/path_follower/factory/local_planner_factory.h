#ifndef LOCAL_PLANNER_FACTORY_H
#define LOCAL_PLANNER_FACTORY_H

#include <path_follower/parameters/local_planner_parameters.h>
#include <path_follower/factory/abstract_factory.h>

#include <memory>
#include <functional>
#include <pluginlib/class_loader.h>

class AbstractLocalPlanner;

/**
 * @brief The LocalPlannerFactory class is responsible for creating instance
 *        of the LocalPlanner class.
 * @see LocalPlanner
 */
class LocalPlannerFactory : public AbstractFactory
{
public:
    /**
     * @brief LocalPlannerFactory
     * @param opt Configuration to use when creating local planners
     */
    LocalPlannerFactory(const LocalPlannerParameters& opt);

    /**
     * @brief makeConstrainedLocalPlanner creates an instance of the LocalPlanner identified by <name>.
     *        It also adds constraints based on the current configuration.
     * @param name The name of the local planner to instanciate
     * @return A shared pointer to the local planner
     */
    std::shared_ptr<AbstractLocalPlanner> makeConstrainedLocalPlanner(const std::string &name);

    /**
     * @brief registerPlanner registers the class <Planner> with the identifier <type>.
     * @param type Identifier for creating instances of type <Planner>
     */
    template <typename Planner>
    static void registerPlanner(const std::string& type)
    {
        planner_constructors_.emplace(toLower(type), [](){
            return std::make_shared<Planner>();
        });
    }

private:
    std::shared_ptr<AbstractLocalPlanner> makeLocalPlanner(const std::string &name);

private:
    const LocalPlannerParameters& opt_;

    pluginlib::ClassLoader<AbstractLocalPlanner> planner_loader_;
    static std::map<std::string, std::function<std::shared_ptr<AbstractLocalPlanner>()>> planner_constructors_;
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
