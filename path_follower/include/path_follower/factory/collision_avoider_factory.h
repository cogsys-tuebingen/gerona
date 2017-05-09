#ifndef COLLISION_AVOIDER_FACTORY_H
#define COLLISION_AVOIDER_FACTORY_H

#include <path_follower/factory/abstract_factory.h>

#include <memory>
#include <string>

class CollisionAvoider;

class CollisionAvoiderFactory : public AbstractFactory
{
public:
    std::shared_ptr<CollisionAvoider> makeObstacleAvoider(const std::string &name);
};


#endif // COLLISION_AVOIDER_FACTORY_H
