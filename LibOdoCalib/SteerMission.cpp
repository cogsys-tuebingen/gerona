
///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

// C/C++
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>

// Workspace
#include "../Gnuplot/Gnuplot.h"

// Project
#include "SteerMission.h"
#include "SteerEstimator.h"

///////////////////////////////////////////////////////////////////////////////
// IMPLEMENTATION
///////////////////////////////////////////////////////////////////////////////

using namespace std;

//// class SteerMission ///////////////////////////////////////////////////////

SteerMission::SteerMission( ConfigFileReader *config, CalibBotInterface *proxy ) {
    // Init member
    mProxy = proxy;

    // Read config parameters
    mForwardSpeed = config->GetDouble( "steerMission::forwardSpeed", 0.3 );
    mReverseSpeed = config->GetDouble( "steerMission::reverseSpeed", 0.3 );
    if ( mReverseSpeed > 0 )
        mReverseSpeed = -mReverseSpeed;
    mSteerMin = M_PI*config->GetDouble( "steerMission::steerMinDeg", 2.5 ) / 180.0;
    mSteerMax = M_PI*config->GetDouble( "steerMission::steerMaxDeg", 20 ) / 180.0;
    mSteerStep = M_PI*config->GetDouble( "steerMission::steerStepDeg", 2.5 ) / 180.0;
    mArcLength = config->GetDouble( "steerMission::arcLength", 1.1 );
    mWheelbase = config->GetDouble( "steerMission::wheelbase", 0.34 );
}

bool SteerMission::execute() {
    double steerReq = mSteerMin; // Requested steer angle
    double speedReq = mForwardSpeed; // Requested speed
    SteerEstimator steerEst( mWheelbase );
    Gnuplot gnuplot;
    Vector3d slamPose;
    Vector3d odoDelta;

    cout << "# RequestedSteer MeasuredSteer Forward" << endl;

    while ( steerReq <= mSteerMax ) {
        mProxy->Read();
        steerEst.Reset();        
        mProxy->GetSlamPose( slamPose );
        steerEst.SetStartingPose( slamPose );
        // Drive forwards/backwards
        while ( steerEst.GetArcLength() < mArcLength ) {
            mProxy->Read();
            if ( mProxy->IsFreshSlamPose()) {
                mProxy->GetSlamPose( slamPose );
                steerEst.AddPoint( slamPose );
            }
            mProxy->SetSpeedSteer( speedReq, steerReq );
            usleep( 5000 );
        }
        // Stop robot
        mProxy->SetSpeedSteer( 0, steerReq );

        // Log/plot data
        stringstream logName;
        logName << (( speedReq > 0 ) ? "SteerMissionForward_" : "SteerMissionReverse_");
        logName << ( 180.0 * steerReq / M_PI ) << "Deg.log";
        steerEst.WriteLogFile( logName.str());
        steerEst.GnuplotCircle( gnuplot, logName.str());

        // Message
        cout << steerReq << " " << steerEst.GetFrontSteer() << " " << ( speedReq > 0 ) << endl;

        // Set new speed/steer angle
        if ( speedReq > 0 ) {
            // Drive backwards
            speedReq = mReverseSpeed;
        } else {
            // Drive forwards and increase steer angle request
            steerReq += mSteerStep;
            speedReq = mForwardSpeed;
        }

        // Wait until the robot stops
        usleep( 500000 );
        while ( fabs( odoDelta[0] ) > 1E-6 ) {
            usleep( 5000 );
        }
    }
}
