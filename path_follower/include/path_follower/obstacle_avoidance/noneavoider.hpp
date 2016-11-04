#ifndef NONEAVOIDER_H
#define NONEAVOIDER_H

#include <path_follower/obstacle_avoidance/obstacleavoider.h>

/**
 * @brief Dummy avoider that does nothing. Use this to deactivate obstacle avoidance.
 */
class NoneAvoider : public ObstacleAvoider
{
public:
    virtual bool avoid(MoveCommand* const, const State &)
    {
        // pass
        return false;
    }
};

#endif // NONEAVOIDER_H
