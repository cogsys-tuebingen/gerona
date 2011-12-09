/**
 (c) Lehrstuhl RA Universitaet Tuebingen

 @author: Marks
 @date 2010

 @file WheelOdometry.h
 */


///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

// C/C++
#include <math.h>
#include <iostream>

// Workspace
#include "Misc.h"

// Project
#include "WheelOdometry.h"

///////////////////////////////////////////////////////////////////////////////
// IMPLEMENTATION
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace Ra;

WheelOdometry::WheelOdometry( double wheelbase, double robotWidth ) {
    SetRobotWheelbase( wheelbase );
    mRobotWidth = robotWidth;
    mPose = Vector3d::Zero();
    mVelocity = Vector3d::Zero();
    mVelocityNorm = 0;
}

void WheelOdometry::Update(
        double frontSteer, double rearSteer,
        double dt,
        double leftEncoder, double rightEncoder ) {
    mSteerModel.SetSteerAngles( frontSteer, rearSteer );
    double averageDist = 0.5 * ( leftEncoder + rightEncoder );
    InternalUpdate( dt, averageDist );
}

void WheelOdometry::Update(
        double frontSteer, double rearSteer,
        double dt,
        double encoder, bool leftEncoder ) {
    mSteerModel.SetSteerAngles( frontSteer, rearSteer );
    double dist;
    if ( mSteerModel.GetTurn() == AckermannSteering::LEFT ) {
        dist = leftEncoder ? mSteerModel.GetRearRadius()*encoder / ( mSteerModel.GetRearRadius() - 0.5*mRobotWidth )
            : mSteerModel.GetRearRadius()*encoder / ( mSteerModel.GetRearRadius() + 0.5*mRobotWidth );
    } else if ( mSteerModel.GetTurn() == AckermannSteering::RIGHT ) {
        dist = leftEncoder ? mSteerModel.GetRearRadius()*encoder / ( mSteerModel.GetRearRadius() + 0.5*mRobotWidth )
            : mSteerModel.GetRearRadius()*encoder / ( mSteerModel.GetRearRadius() - 0.5*mRobotWidth );
    } else {
        dist = encoder;
    }
    InternalUpdate( dt, dist );
}

void WheelOdometry::InternalUpdate( const double &dt, const double &dist ) {
    mDeltaPose = Vector3d::Zero();
    // No movement?
    if ( fabs( dist ) < 1E-6 ) {
        mVelocity = Vector3d::Zero();
        mVelocityNorm = 0;
        return;
    }

    // Straight line?
    if ( mSteerModel.GetTurn() == AckermannSteering::NONE ) {
        mDeltaPose[0] = dist;
        UpdatePose( mDeltaPose, dt );
        return;
    }

    // Parallel steer?
    if ( mSteerModel.GetTurn() == AckermannSteering::PARALLEL ) {
        mDeltaPose[0] = dist * cos( fabs( mSteerModel.GetFrontSteer()));
        if ( dist < 0 )
            mDeltaPose[1] = -mDeltaPose[1];
        UpdatePose( mDeltaPose, dt );
        return;
    }

    // We are turning. Compute delta coordinates
    double phi = dist / mSteerModel.GetRearRadius();
    if ( mSteerModel.GetTurn() == AckermannSteering::RIGHT )
        phi = -phi;
    Vector2d center;
    mSteerModel.GetIcpCoordinates( center );
    Vector3d deltaPose;
    mDeltaPose[0] = center[0] * ( cos( center[1] + phi ) - cos( center[1] ));
    mDeltaPose[1] = center[0] * ( sin( center[1] + phi ) - sin( center[1] ));
    mDeltaPose[2] = phi;
    UpdatePose( mDeltaPose, dt );
}

void WheelOdometry::UpdatePose( const Vector3d &deltaPose, const double &dt ) {
    // Compute new real world position
    double c = cos( mPose[2] );
    double s = sin( mPose[2] );
    Vector3d oldPose = mPose;
    mPose[0] += ( c*deltaPose[0] - s*deltaPose[1] );
    mPose[1] += ( s*deltaPose[0] + c*deltaPose[1] );
    double heading = Misc::normalizeAngle(mPose[2] + deltaPose[2]);
    mPose[2] = (float)heading;
    Vector3d diff = mPose - oldPose;
    
    // Compute velocity
    if ( dt > 0 ) {
        mVelocity = ( mPose - oldPose ) / dt;
   	    mVelocityNorm = sqrt( mVelocity[0]*mVelocity[0] + mVelocity[1]*mVelocity[1]);
        if ( deltaPose[0] < 0 )
            mVelocityNorm = -mVelocityNorm;
    }
}

void WheelOdometry::SetPose( const Vector3d &pose ) {
    mPose = pose;
}

void WheelOdometry::GetPose( Vector3d &pose ) const {
    pose = mPose;
}

void WheelOdometry::GetDeltaPose( Vector3d &deltaPose ) const {
    deltaPose = mDeltaPose;
}

void WheelOdometry::GetVelocity( Vector3d &velo ) const {
    velo = mVelocity;
}

double WheelOdometry::GetVelocity() const {
    return mVelocityNorm;
}

double WheelOdometry::GetRobotWheelbase() const {
    return mSteerModel.GetWheelbase();
}

void WheelOdometry::SetRobotWheelbase(const double &wheelbase) {
    mSteerModel.SetWheelbase( wheelbase );
}

double WheelOdometry::GetRobotWidth() const {
    return mRobotWidth;
}

void WheelOdometry::SetRobotWidth( const double &robotWidth ) {
    mRobotWidth = robotWidth;
}
