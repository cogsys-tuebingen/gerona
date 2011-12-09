#ifndef DISTANCETRACKER_H
#define DISTANCETRACKER_H

#include "RobotProxy.h"
#include "Eigen/Core"

using namespace Eigen;


class DistanceTracker
{
public:

    DistanceTracker( bool useIcp = false );

    void Update( RobotProxy *robot );

    /**
     * @return The travelled distance. Attention: Always positive.
     */
    float GetDistance() const;

    void Reset();

private:
    float mDist;
    bool  mUseIcp;
};

#endif // DISTANCETRACKER_H
