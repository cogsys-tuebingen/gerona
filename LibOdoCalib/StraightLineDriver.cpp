
////////////////////////////////////////////////////////////////////////////////
// I N C L U D E S
////////////////////////////////////////////////////////////////////////////////

// C/C++
#include <cmath>
#include <iostream>

// Project
#include "StraightLineDriver.h"

////////////////////////////////////////////////////////////////////////////////
// I M P L E M E N T A T I O N
////////////////////////////////////////////////////////////////////////////////

using namespace std;

StraightLineDriver::StraightLineDriver( CalibBotInterface *proxy )
    : mProxy( proxy ), mStartPose( Vector3d::Zero()), mLength( 0 ), mSpeed( 0 ),
    mSteer( 0 ), mSteerP( 1.3 ), mSteerI( 0.5 ), mZeroForw( 0 ), mZeroRev( 0 ),
    mSteerSum( 0 ), mSteerCount( 0 ) {
    mUpdateWatch.restart();
}

bool StraightLineDriver::Update() {
    Vector3d pose;

    // No fresh SLAM pose available?
    if ( !mProxy->IsFreshSlamPose()) {
        // Not possible to check line length and to compute new steer angle etc
        mProxy->SetSpeedSteer( mSpeed, mSteer );
        return false;
    }

    // Get SLAM pose
    mProxy->GetSlamPose( pose );

    // Compute new steer angle
    mSteer = mStartPose[2] - pose[2];
    while ( mSteer > M_PI )
       mSteer -= 2 * M_PI;
    while ( mSteer < -M_PI )
       mSteer += 2 * M_PI;
    mSteerErrorI += mSteer * mUpdateWatch.sElapsed();
    mUpdateWatch.restart();

    // Simple pi-control (reverse if we are driving backwards)
    mSteer = mSteerP * mSteer + mSteerI * mSteerErrorI;
    if ( mSpeed < 0 )
        mSteer = -(mSteer - mZeroRev);
    else
        mSteer += mZeroForw;

    // Limit steer angle
    if ( fabs( mSteer ) > 0.23 )
        mSteer = mSteer > 0 ? 0.23 : -0.23;

    // Compute travelled distance
    double dist = sqrt( pow( pose[0] - mStartPose[0], 2 ) + pow( pose[1] - mStartPose[1], 2 ));
    // Requested distance reached?
    if ( dist >= mLength ) {
        mProxy->SetSpeedSteer( 0.0, mSteer );
        // Wait until robot stops
        if ( fabs( mProxy->GetOdoSpeedFiltered()) > 1E-3 ) {
            return false;
        }
        // Distance reached and the robot was brought to a stop. We are done.
        cout << "Zero forw: " << mZeroForw << " zero rev: " << mZeroRev << endl;
        if ( mSpeed > 0 )
            mZeroForw = mSteerSum / mSteerCount;
        else
            mZeroRev = mSteerSum / mSteerCount;
        return true;
    }

    // Set speed/steer
    mProxy->SetSpeedSteer( mSpeed, mSteer );
    //cout << "#mark " << mSpeed << " " << pose[2] << " " << mSteer << endl;

    // Add new steer angle to steer sum
    mSteerSum += mSteer;
    mSteerCount++;

    // Distance not reached
    return false;
}

void StraightLineDriver::SetStart( const Vector3d &pose ) {
    mStartPose = pose;
    mSteerErrorI = 0;
    mSteerSum = 0;
    mSteerCount = 0;
    mUpdateWatch.restart();
}

void StraightLineDriver::SetLength( double length ) {
    mLength = length;
}

void StraightLineDriver::SetSpeed( double speed ) {
    mSpeed = speed;
}
