
///////////////////////////////////////////////////////////////////////////////
// I N C L U D E S
///////////////////////////////////////////////////////////////////////////////

// C/C++
#include <cmath>
#include <iostream>

// Project
#include "EncoderEstimator.h"

///////////////////////////////////////////////////////////////////////////////
// I M P L E M E N T A T I O N
///////////////////////////////////////////////////////////////////////////////

using namespace std;
// 128 ticks per turn for street tires
// 144 for trekker
EncoderEstimator::EncoderEstimator()
    : mStartPose( Vector3d::Zero()), mLastPose(Vector3d::Zero()), mCalibLeft( 0 ), mCalibRight( 0 ),
    mDist( 0 ),mDeltaDist(1.0), mStartTicksLeft( 0 ), mStartTicksRight( 0 ), mTicksPerTurn(144) {
}

void EncoderEstimator::Update( const Vector3d &pose,
                               int ticksLeft, int ticksRight ) {
    // Compute dist and calibration
    mDist = sqrt( pow( pose[0] - mStartPose[0], 2 ) + pow( pose[1] - mStartPose[1], 2 ));
    mCalibLeft = mDist / (double)(abs( ticksLeft - mStartTicksLeft ));
    mCalibRight = mDist / (double)(abs( ticksRight - mStartTicksRight ));
    double lastDist = sqrt( pow( pose[0] - mLastPose[0], 2 ) + pow( pose[1] - mLastPose[1], 2 ));
    if (lastDist>mDeltaDist) {
        cout << "Left Start: " << mStartTicksLeft << " Current: " << ticksLeft << endl;
        cout << "Right Start: " << mStartTicksRight << " Current: " << ticksRight << endl;
        cout << "Distance: " << mDist << " m" << endl;
        cout << "calibleft:"<<mCalibLeft<< " calibright:"<<mCalibRight<<endl;
        cout << "Wheel Diameter Left" << (mTicksPerTurn*mCalibLeft)/M_PI << "m"<<endl;
        cout << "Wheel Diameter Right" << (mTicksPerTurn*mCalibRight)/M_PI << "m"<<endl;

        mLastPose=pose;
    }
}

double EncoderEstimator::GetDistance() const {
    return mDist;
}

double EncoderEstimator::GetCalibrationLeft() const {
    return mCalibLeft;
}

double EncoderEstimator::GetCalibrationRight() const {
    return mCalibRight;
}

void EncoderEstimator::SetStart( const Vector3d &pose, int ticksLeft, int ticksRight ) {
    mStartPose = pose;
    mLastPose = pose;
    mStartTicksLeft = ticksLeft;
    mStartTicksRight = ticksRight;
    mCalibLeft = mCalibRight = 0;
    mDist = 0;
}
