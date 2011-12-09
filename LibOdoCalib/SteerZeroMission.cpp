
#include "SteerZeroMission.h"
#include <math.h>
#include <iostream>

using namespace std;

SteerZeroMission::SteerZeroMission( RobotProxy *robot )
    : mRobot( robot ), mDist( true )
{
    mSpeed = 0.2f;
    mSteerRad = 0.0f;
    mDone = false;
}

bool SteerZeroMission::Execute()
{
    mDist.Update( mRobot );
    if ( mDist.GetDistance() > 1.0 ) {
        mSpeed = 0;
        mDone = true;
    }
    mRobot->SetCarlike( mSpeed, mSteerRad );

    return true;
}

bool SteerZeroMission::Done()
{
    return mDone;
}
