/**
 (c) Lehrstuhl RA Universitaet Tuebingen

 @author: Marks
 @date 2010

 @file AckermannSteering.cpp
 */


///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

// C/C++
#include <cmath>
#include <iostream>

// Project
#include "AckermannSteering.h"

///////////////////////////////////////////////////////////////////////////////
// MAKROS
///////////////////////////////////////////////////////////////////////////////

#define SIGN(VALUE__) (VALUE__ >= 0 ? 1 : -1 )

///////////////////////////////////////////////////////////////////////////////
// IMPLEMENTATION
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace Ra;

AckermannSteering::AckermannSteering() {
    mWheelbase = 0.34;
    mSteerFront = mSteerRear = 0;
    mFrontRadius = mCenterRadius = mRearRadius = 0;
    mTurn = NONE;
    //mCenter = Vector2d::Zero();
    mCenterX = mCenterY = 0.0;
}

void AckermannSteering::calcIcr(const Vector2d &p0, const Vector2d &p1,
                           const Vector2d&p2,double wb,Vector2d &m, double &r, double &deltaf)
{
  // calc circle through 3 points
  // http://2000clicks.com/mathhelp/GeometryConicSectionCircleEquationGivenThreePoints.aspx
  double a,b,c,d,e,f;
  a=p0.x();
  b=p0.y();
  c=p1.x();
  d=p1.y();
  e=p2.x();
  f=p2.y();
  m.x()= (0.5)*((a*a+b*b)*(f-d) + (c*c+d*d)*(b-f) + (e*e+f)*(d-b)) / (a*(f-d)+c*(b-f)+e*(d-b));
  m.y()= (0.5)*((a*a+b*b)*(e-c) + (c*c+d*d)*(a-e) + (e*e+f*f)*(c-a)) / (b*(e-c)+d*(a-e)+f*(c-a));


  r=(p0-m).norm();
  deltaf=atan(wb/r);
  if (m.y()<0) {
    // right curve
    deltaf=fabs(deltaf)*-1.0;
  } else {
    deltaf=fabs(deltaf);
  }
}


void AckermannSteering::SetSteerAngles( const double &front, const double &rear ) {

    mSteerFront = front;
    mSteerRear = rear;

    double steerFrontAbs = fabs( mSteerFront );
    double steerRearAbs = fabs( mSteerRear );

    // Check steer angle values
    if ( steerFrontAbs >= 0.5*M_PI || steerRearAbs >= 0.5*M_PI ) {
        // This should never happen
        cout    << "AckermannSteering: ERROR. Steer angle is too large.\n"
                << "AckermannSteering: Setting steer angles to zero." << endl;
        mSteerFront = mSteerRear = 0;
    }

    // Are we turning?
    if ( fabs( mSteerFront - mSteerRear ) < 1E-6 ) {
        // Parallel steering
        mTurn = steerFrontAbs < 1E-6 ? NONE : PARALLEL;
        mFrontRadius = mCenterRadius = mRearRadius = 0;
        //mCenter = Vector2d::Zero();
        mCenterX = mCenterY = 0.0;
        return;
    }

    // Steer angles differ from each other => we are turning.
    // Determine turning direction and compute triangle angles
    double gF, gR, gC;
    if ( steerFrontAbs >= steerRearAbs ) {
        mTurn = mSteerFront > 0 ? LEFT : RIGHT;
        gF = 0.5*M_PI - steerFrontAbs;
        gR = SIGN( mSteerFront ) == SIGN( mSteerRear ) ?
             0.5*M_PI + steerRearAbs :
             0.5*M_PI - steerRearAbs;
    } else {
        mTurn = mSteerRear > 0 ? RIGHT : LEFT;
        gR = 0.5*M_PI - steerRearAbs;
        gF = SIGN( mSteerFront ) == SIGN( mSteerRear ) ?
             0.5*M_PI + steerFrontAbs :
             0.5*M_PI - steerFrontAbs;
    }
    gC = M_PI - gF - gR;

    // Compute turning radius
    mFrontRadius = mWheelbase * sin( gR ) / sin( gC );
    mRearRadius = mWheelbase * sin( gF ) / sin( gC );
    mCenterRadius = sqrt(
            0.25*pow( mWheelbase, 2 ) + pow( mRearRadius, 2 )
            - mWheelbase*mRearRadius*cos( gR ));

    // Compute ICP coordinates
    double wheelbaseSq = mWheelbase*mWheelbase;
    double rearRadiusSq = mRearRadius*mRearRadius;
    double centerRadiusSq = mCenterRadius*mCenterRadius;
    double triangleH = sqrt( 0.5*wheelbaseSq*rearRadiusSq
                             + 2.0*rearRadiusSq*centerRadiusSq
                             + 0.5*centerRadiusSq*wheelbaseSq
                             - ( 0.0625*wheelbaseSq*wheelbaseSq
                                 + rearRadiusSq*rearRadiusSq
                                 + centerRadiusSq*centerRadiusSq ))
                       / mWheelbase;
    double phiC;
    if ( fabs( mSteerFront + mSteerRear ) < 1E-6 ) {
        phiC = 0;
    } else {
        phiC = acos( triangleH / mCenterRadius );
    }
    if ( fabs( mSteerFront ) >= fabs( mSteerRear )) {
        phiC = 0.5*M_PI - phiC;
    } else {
        phiC = 0.5*M_PI + phiC;
    }
    //mCenter[0] = mCenterRadius; // Radius
    //mCenter[1] = mTurn == LEFT ? 2.0*M_PI - phiC : phiC; // Angle
    mCenterX = mCenterRadius;
    mCenterY = mTurn == LEFT ? 2.0*M_PI - phiC : phiC; // Angle

    // Error checking
   // if ( isnan( mCenter[0] ) || isnan( mCenter[1] )) {
     if ( isnan( mCenterX ) || isnan( mCenterY )) {
        cout << "AckermannSteering: ERROR. Computed invalid ICP coordinates. Ignoring steer angles." << endl;
        // Ignore steer angles
        mTurn = NONE;
        //mCenter = Vector2d::Zero();
        mCenterX = mCenterY = 0.0;
    }   
}

void AckermannSteering::SetSteerAnglesDeg( const double &frontDeg, const double &rearDeg ) {
    SetSteerAngles( 180.0 * frontDeg / M_PI, 180.0 * rearDeg / M_PI );
}

void AckermannSteering::SetWheelbase( const double &wheelbase ) {
    mWheelbase = wheelbase;
    SetSteerAngles( mSteerFront, mSteerRear );
}
