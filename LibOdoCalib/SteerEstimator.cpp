
// C/C++
#include <math.h>
#include <iostream>
#include <fstream>
#include <sstream>

// Project
#include "SteerEstimator.h"

using namespace std;

SteerEstimator::SteerEstimator( const double &wheelbase ) {
    mWheelbase = wheelbase;
    Reset();
}

void SteerEstimator::AddPoint( const Vector3d &point ) {
    mCircleEst.AddPoint( point[0], point[1],0 );
    mCircleEst.Update();
    mEstPossible = mCircleEst.GetRadius() > 0;
    mEndPose = point;
    if ( mEstPossible ) {
        mArcLength = mCircleEst.GetArcLength();
        // Transform coordinate system (origin equals start pose)
        // Translation
        Vector2d circleCenter = mCircleEst.GetCenter();
        circleCenter[0] -= mStartPose[0];
        circleCenter[1] -= mStartPose[1];
        // Rotation
        Matrix2d rot;
        rot << cos( mStartPose[2] ), -sin( mStartPose[2] ),
               sin( mStartPose[2] ), cos( mStartPose[2] );
        circleCenter = rot * circleCenter;

        // Compute steer angles
        double rearRadius = sqrt( pow( mCircleEst.GetRadius(), 2 ) - pow( 0.5*mWheelbase, 2 ));
        mFrontSteer = 0.5*M_PI -fabs( atan( rearRadius / mWheelbase ));
        if ( circleCenter[1] < 0 )
            mFrontSteer = -mFrontSteer;
        mBackSteer = atan( circleCenter[0] / circleCenter[1] ); // TODO Compute sign
    }
}

bool SteerEstimator::WriteLogFile( const std::string &name ) {
    // Open stream
    ofstream out( name.c_str());
    if ( !out ) {
        cout << "# ERROR: Cannot write logfile\"" << name << "\"." << endl;
        return false;
    }
    // Write radius, center point, computed steer angle etc
    Vector2d point = mCircleEst.GetCenter();
    out << "# SteerEstimator logfile.\n"
            << "# Henrik Marks 2010\n"
            << "#\n"
            << "# Circle radius [m]: " << mCircleEst.GetRadius() << "\n"
            << "# Circle center: " << point[0] << ", " << point[1] << "\n"
            << "# Start pose: " << mStartPose[0] << ", " << mStartPose[1] << ", " << mStartPose[2] << "\n"
            << "# Estimated front steer [rad]: " << mFrontSteer << "\n"
            << "# Estimated rear steer [rad]: " << mBackSteer << endl;
    // Write trajectory points
    out << "# Trajectory points (x, y):\n";
    const Vector3dList * pointList = mCircleEst.GetPointList();
    Vector3dList::const_iterator pointIter = pointList->begin();
    for ( ; pointIter != pointList->end(); pointIter++ )
        out << (*pointIter)[0] << "\t" << (*pointIter)[1] << "\n";
    out << endl;
    out.close();
    return true;
}

bool SteerEstimator::GnuplotCircle( Gnuplot &gnuplot, const string &logFile ) {
    Vector2d center = mCircleEst.GetCenter();

    gnuplot.cmd( "set grid" );
    gnuplot.cmd( "set parametric" );
    double tStart = ComputePhi( mStartPose[0] - center[0], mStartPose[1] - center[1] );
    double tEnd = ComputePhi( mEndPose[0] - center[0], mEndPose[1] - center[1]);
    ostringstream tRangeCmd;
    tRangeCmd << "set trange [";
    if ( tEnd < tStart )
        tRangeCmd << tEnd << ":" << tStart;
    else
        tRangeCmd << tStart << ":" << tEnd;
    tRangeCmd << "]";
    gnuplot.cmd( tRangeCmd.str());
    ostringstream plotCmd;
    plotCmd << "plot " << mCircleEst.GetRadius() << " * cos(t) + " << center[0]
            << ", " << mCircleEst.GetRadius() << " * sin(t) + " << center[1] << " t \"Estimated circle\","
            << "'" << logFile << "' u 1:2 w lp t \"ICP trajectory\"";
    gnuplot.cmd( plotCmd.str());

    return true;
}

void SteerEstimator::Reset() {
    mCircleEst.Reset();
    mArcLength = mFrontSteer = mBackSteer = 0;
    mStartPose = mEndPose = Vector3d::Zero();
    mEstPossible = false;
}

double SteerEstimator::ComputePhi( const double &x, const double &y ) {
    if ( x > 0 ) {
        return atan( y / x );
    }
    if ( x == 0 ) {
        return ( x > 0 ? 0.5*M_PI : -0.5*M_PI );
    }
    if ( y >= 0 ) {
        return atan( y / x ) + M_PI;
    }
    return atan( y / x ) - M_PI;
}
