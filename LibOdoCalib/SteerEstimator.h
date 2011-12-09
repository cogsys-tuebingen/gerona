#ifndef STEERESTIMATOR_H
#define STEERESTIMATOR_H

// C/C++
#include <string>

// Workspace
#include "../Gnuplot/Gnuplot.h"

// Project
#include "CircleEstimator.h"

// Eigen
#include "Eigen/Core"
using namespace Eigen;


class SteerEstimator {

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SteerEstimator( const double &wheelbase );

    void SetWheelbase( const double &wheelbase ) { mWheelbase = wheelbase; }
    void SetStartingPose( const Vector3d &start ) { mStartPose = start; }
    void AddPoint( const Vector3d &point );
    void Reset();

    double GetArcLength() const { return mArcLength; }
    bool IsEstimationPossible() const { return mEstPossible; }
    double GetFrontSteer() const { return mFrontSteer; }
    double GetBackSteer() const { return mBackSteer; }
    bool WriteLogFile( const std::string &name );
    bool GnuplotCircle( Gnuplot &gnuplot, const std::string &logFile );

private:

    double ComputePhi( const double &x, const double &y );

    CircleEstimator mCircleEst;
    double mWheelbase;
    double mArcLength;
    bool mEstPossible;
    double mFrontSteer;
    double mBackSteer;
    Vector3d mStartPose;
    Vector3d mEndPose;
};

#endif // STEERESTIMATOR_H
