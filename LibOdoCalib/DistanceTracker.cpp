
#include "DistanceTracker.h"
#include <math.h>

DistanceTracker::DistanceTracker( bool useIcp )
{
    mUseIcp = useIcp;
    Reset();
}

void DistanceTracker::Reset()
{
    mDist = 0;
}

void DistanceTracker::Update( RobotProxy *robot )
{
    Vector3f deltaPose;
    if ( mUseIcp ) {
        robot->GetIcpDeltaPose( deltaPose );
    } else {
        robot->GetDeltaPose( deltaPose );
    }
    mDist += sqrt( deltaPose[0]*deltaPose[0] + deltaPose[1]*deltaPose[1]);
}

float DistanceTracker::GetDistance() const
{
    return mDist;
}
