#ifndef COLLISION_AVOIDER_FACTORY_H
#define COLLISION_AVOIDER_FACTORY_H

#include <path_follower/factory/abstract_factory.h>

#include <memory>
#include <string>

class CollisionAvoider;

/**
 * @brief The CollisionAvoiderFactory class is responsible for creating instance
 *        of the CollisionAvoider class.
 * @see CollisionAvoider
 */
class CollisionAvoiderFactory : public AbstractFactory
{
public:
    /**
     * @brief makeObstacleAvoider creates an instance of the CollisionAvoider identified by <name>.
     * @param name The name of the collision avoider to instanciate
     * @return A shared pointer to the avoider
     */
    std::shared_ptr<CollisionAvoider> makeObstacleAvoider(const std::string &name);
};


#endif // COLLISION_AVOIDER_FACTORY_H
