
///////////////////////////////////////////////////////////////////////////////
// I N C L U D E S
///////////////////////////////////////////////////////////////////////////////

// C/C++
#include <iostream>
#include <cmath>

// Project/workspace
#include <MathHelper.h>
#include "EncoderMission.h"
#include "EncoderEstimator.h"

// Eigen
#include "Eigen/Core"
using namespace Eigen;


///////////////////////////////////////////////////////////////////////////////
// I M P L E M E N T A T I O N
///////////////////////////////////////////////////////////////////////////////

using namespace std;

EncoderMission::EncoderMission( ConfigFileReader *config, CalibBotInterface *proxy )
    : mProxy( proxy ), mLineDriver( proxy )
{
    // Read config
    mNumRuns = config->GetInt( "encoderMission::numberOfRuns", 8 );
    mMinDist = config->GetDouble( "encoderMission::minDistance", 2.0 ); //3 );
    mSpeed = fabs( config->GetDouble( "encoderMission::speed", 0.4 ));

    // Set line driver config
    mLineDriver.SetLength( mMinDist );
    mLineDriver.SetSpeed( mSpeed );
}

bool EncoderMission::execute() {
    Vector3d slamPose;
    vector<double> leftCalib;
    vector<double> rightCalib;

    // For each run
    for ( int i = 0; i < mNumRuns; ++i ) {
        mProxy->Read(); // Read incoming messages

        // Get/set start pose of encoder calibration estimator
        mProxy->GetSlamPose( slamPose );
        mLineDriver.SetStart( slamPose );
        mEncEst.SetStart( slamPose, mProxy->GetLeftEncoderTicks(), mProxy->GetRightEncoderTicks());

        // Wait until minimum distance is reached
        while ( true ) {
            mProxy->Read();
            // New SLAM pose?
            if ( !mProxy->IsFreshSlamPose()) {
                usleep( 1000 ); // Wait 1 msec
                continue;
            }

            // New SLAM pose available
            mProxy->GetSlamPose( slamPose );
            mEncEst.Update( slamPose, mProxy->GetLeftEncoderTicks(), mProxy->GetRightEncoderTicks());

            // Update line driver, check if distance was reached and the robot stopped.
            if ( mLineDriver.Update()) {
                // Get calibration data
                leftCalib.push_back( mEncEst.GetCalibrationLeft());
                rightCalib.push_back( mEncEst.GetCalibrationRight());
                cout << "Encoder mission run " << i << " of " << mNumRuns << " done." << endl;

                // Set new speed (forward/reverse?)
                if (( i + 2 ) % 2 != 0 ) {
                    mLineDriver.SetSpeed( mSpeed );
                } else {
                    mLineDriver.SetSpeed( -mSpeed );
                }

                break;
            }
        }
    }

    // Print raw results
    cout << "\nRun #\tLeft [m/Ticks]\tRight [m/Ticks]" << endl;
    for ( size_t i = 0; i < leftCalib.size(); ++i ) {
        cout << i << "\t" << leftCalib[i] << "\t" << rightCalib[i] << endl;
    }

    // Compute and print average etc
    double leftMean = mean1D( leftCalib );
    double rightMean = mean1D( rightCalib );
    double leftStdDev = standardDeviation1D( leftCalib );
    double rightStdDev = standardDeviation1D( rightCalib );

    cout << "_____________________________________________________\n\n"
         << "        E N C O D E R    C A L I B R A T I O N\n"
         << "_____________________________________________________\n\n"
         << "Mean left/right [m/Ticks]: " << leftMean << " " << rightMean << "\n"
         << "Std deviation left/right : " << leftStdDev << " " << rightStdDev << "\n"
         << "_____________________________________________________\n"
         << endl;

    return true;

}
