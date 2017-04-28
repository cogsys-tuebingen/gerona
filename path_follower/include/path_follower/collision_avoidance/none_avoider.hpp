#ifndef NONE_AVOIDER_H
#define NONE_AVOIDER_H

#include <path_follower/collision_avoidance/collision_avoider.h>

/**
 * @brief Dummy avoider that does nothing. Use this to deactivate obstacle avoidance.
 */
class NoneAvoider : public CollisionAvoider
{
public:
    virtual bool avoid(MoveCommand* const, const State &)
    {
        // pass
        return false;
    }
};

#endif // NONEAVOIDER_H
