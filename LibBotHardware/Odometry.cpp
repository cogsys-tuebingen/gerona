/*
 * RamaxxOdometry.cpp
 *
 *  Created on: Feb 1, 2010
 *      Author: marks, bohlmann
 */

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

// C/C++
#include <algorithm>
#include <string.h>
#include <iostream>
#include <cmath>

// Workspace
#include <Misc.h>

// Project
#include "Odometry.h"
#include "MsgPrint.h"

///////////////////////////////////////////////////////////////////////////////
// IMPLEMENTATION
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace Ra;

//// class RamaxxOdoemtry /////////////////////////////////////////////////////////

RamaxxOdometry::RamaxxOdometry( RobotSensors *sensors, SteerServo * frontSteer, SteerServo * rearSteer ) {
    mSensors = sensors;
    mRawWheelOdo.SetRobotWidth(0.325);
    mRawWheelOdo.SetRobotWheelbase(0.34);
    mWheelOdo.SetRobotWheelbase( 0.34 );
    mWheelOdo.SetRobotWidth( 0.325 );
    mHeadingError = 0;
    mPose = mVelo = mRawPose = Vector3d::Zero();
    mVelNorm = 0;
    mSteerFront = frontSteer;
    mSteerRear = rearSteer;

    // Rear wheelencoder available?
    if ( !mSensors->hasRearWheelEncoder()) {
        ERRORPRINT( "Odometry: Rear wheel encoder not available" );
        ERRORPRINT( "Odometry will report meaningless data" );
    } else {
        mSensors->getRearWheelEncoder()->addDataListener( this );
    }

    // PNI compass available?
    if ( !mSensors->hasPniCompass()) {
        //player: WARNPRINT( "No PNI spacepoint compass available! Odometry might be inaccurate." );
    } else {
        // Register to receive PNI compass update
        // If the PNI compass reports an error, we will use the i2c compass
        mSensors->getPniCompass()->addDataListener( this );
    }

    // I2c compass available?
    if ( !mSensors->hasI2cCompass()) {
        //player: WARNPRINT( "No i2c compass available! Odometry might be inaccurate." );
    } else {
        mSensors->getI2cCompass()->addDataListener( this );
    }
}

void RamaxxOdometry::dataChanged( const DataChangeEvent &event ) {

    if ( event.owner == mSensors->getRearWheelEncoder()) {
        processWheelEncoderEvent( mSensors->getRearWheelEncoder(), event );
        return;
    }

    if ( event.owner == mSensors->getPniCompass()) {
        processPniCompassEvent( mSensors->getPniCompass(), event );
        return;
    }

    if ( event.owner == mSensors->getI2cCompass()) {
        processI2cCompassEvent( mSensors->getI2cCompass(), event );
    }
}

void RamaxxOdometry::processI2cCompassEvent( Compass *compass, const DataChangeEvent &event ) {
    // Skip data if there is a valid PNI compass
    if ( mSensors->hasPniCompass() && mSensors->getPniCompass()->hasValidData())
        return;

    //DEBUGOUT << "Processing i2c compass data. Heading: " << compass->getHeading() << endl;
    if ( compass->getSensorState() == Sensor::VALID && event.eventType == DataChangeEvent::NEW_DATA )
        correctHeading( compass );
}

void RamaxxOdometry::processPniCompassEvent( Compass *compass, const DataChangeEvent &event ) {
    if ( event.eventType == DataChangeEvent::STATE ) {
        // Use the I2c compass if the PNI sensor reports an error
        if ( compass->getSensorState() == Sensor::ERROR || compass->getSensorState() == Sensor::TIMEOUT ) {
            WARNPRINT( "Odometry uses i2c compass (PNI compass reported an error)." );
            return;
        }
        // PNI compass is valid
        //player: INFOPRINT( "Odometry uses PNI spacepoint compass" );
        return;
    }

    if ( compass->getSensorState() == Sensor::VALID && event.eventType == DataChangeEvent::NEW_DATA ) {
        //DEBUGOUT << "Processing PNI compass data. Heading: " << compass->getHeading() << endl;
        // Correct heading
        correctHeading( compass );
    }
}

void RamaxxOdometry::processWheelEncoderEvent( WheelEncoder *encoder, const DataChangeEvent &event ) {

    switch ( event.eventType ) {
    case DataChangeEvent::STATE:
        if ( encoder->getSensorState() == Sensor::ERROR || encoder->getSensorState() == Sensor::TIMEOUT ) {
            setValidData( false );
            ERRORPRINT( "No valid wheelencoder data available. Odometry data will be inaccurate." );
        } else {
            //INFOPRINT( "Valid wheelencoder data available. Odometry system should work properly." );
        }
        return;

    case DataChangeEvent::NEW_DATA:
        // Process new distance data
        mWheelOdo.SetPose( mPose );
        mRawWheelOdo.SetPose(mRawPose);
        if ( encoder->isLeftEnabled() && encoder->isRightEnabled()) {
            // Use both encoder
            mWheelOdo.Update(
                    mSteerFront->getAngleDeg() * M_PI / 180.0,
                    mSteerRear->getAngleDeg() * M_PI / 180.0,
                    encoder->getTimeDifference(),
                    encoder->getLeftDistance(),
                    encoder->getRightDistance());
            mRawWheelOdo.Update(
                    mSteerFront->getAngleDeg() * M_PI / 180.0,
                    mSteerRear->getAngleDeg() * M_PI / 180.0,
                    encoder->getTimeDifference(),
                    encoder->getLeftDistance(),
                    encoder->getRightDistance());

        } else if ( encoder->isLeftEnabled() ){
            // Use left encoder
            mWheelOdo.Update(
                mSteerFront->getAngleDeg() * M_PI / 180.0,
                mSteerRear->getAngleDeg() * M_PI / 180.0,
                encoder->getTimeDifference(),
                encoder->getLeftDistance(),
                true );
            mRawWheelOdo.Update(
                mSteerFront->getAngleDeg() * M_PI / 180.0,
                mSteerRear->getAngleDeg() * M_PI / 180.0,
                encoder->getTimeDifference(),
                encoder->getLeftDistance(),
                true );
        } else if ( encoder->isRightEnabled()) {
            // Use right encoder
            mWheelOdo.Update(
                mSteerFront->getAngleDeg() * M_PI / 180.0,
                mSteerRear->getAngleDeg() * M_PI / 180.0,
                encoder->getTimeDifference(),
                encoder->getRightDistance(),
                false );
            mRawWheelOdo.Update(
                mSteerFront->getAngleDeg() * M_PI / 180.0,
                mSteerRear->getAngleDeg() * M_PI / 180.0,
                encoder->getTimeDifference(),
                encoder->getRightDistance(),
                false );

        } else {
            // No valid encoder data!
            if ( hasValidData()) {
                ERRORPRINT( "No valid wheelencoder data. Odometry will fail!" );
                return;
            }
        }

        // Comptute travelled distance
        Vector3d newPose;
        mWheelOdo.GetPose( newPose );

        // Error checking. There is the suspicion that odometry reports invalid data
        if ( isnan( newPose[0]) || isnan( newPose[1] ) || isnan( newPose[2] )) {
            //player: ERRORPRINT4( "Odometry computed invalid data (NaN). Skipping. Steer angles: %f %f Encoder: %f %f",
            //             mSteerFront->getAngleDeg(), mSteerRear->getAngleDeg(),
            //             encoder->getLeftDistance(), encoder->getRightDistance());
            setValidData( false );
            return;
        }

        // Moved more than 25 m in one timestep? There is something wrong!
        newPose -= mPose;
        float deltaDist = newPose.norm();
        if ( deltaDist > 25 ) {
            ERRORPRINT4( "Odometry computed inprobable data. Skipping. Steer angles: %f %f Encoder: %f",
                         mSteerFront->getAngleDeg(), mSteerRear->getAngleDeg(),
                         encoder->getLeftDistance(), encoder->getRightDistance());
            setValidData( false );
            return;
        }

        // Everything ok. Set data
        mHeadingError += deltaDist * ODO_VARIANCE_SYS;
        mWheelOdo.GetPose( mPose );
        mWheelOdo.GetVelocity( mVelo );
        mVelNorm = mWheelOdo.GetVelocity();
        mRawWheelOdo.GetPose(mRawPose);
        setValidData( true );
        fireDataChanged();
        break;
    }

}

void RamaxxOdometry::correctHeading( Compass * compass ) {
    if ( !compass->hasValidData()) { // TODO Remove? Should never happen
        return; // Dont use invalid data
    }

    double compassRad = Misc::normalizeAngle(compass->getHeading());

    // Kalman gain
    double k = mHeadingError / ( mHeadingError + compass->getVariance());
    double radDiff = compassRad - mPose[2];
    while ( radDiff > M_PI )
       radDiff -= 2 * M_PI;
    while ( radDiff < -M_PI )
       radDiff += 2 * M_PI;
    // Correction
    mPose[2] = mPose[2] + k * radDiff;
    mHeadingError *= ( 1 - k );
    fireDataChanged();
}

void RamaxxOdometry::setPosition( const Vector3d &pose ) {
    mPose = pose;
    mRawPose = pose;
    fireDataChanged();
}

void RamaxxOdometry::getPosition( Vector3d &pose ) const {
   pose = mPose;
}

void RamaxxOdometry::getRawPosition( Vector3d &pose ) const {
   pose = mRawPose;
}


void RamaxxOdometry::getVelocity( Vector3d &vel ) const {
    vel = mVelo;
}

double RamaxxOdometry::getVelocity() const {
    return mVelNorm;
}

double RamaxxOdometry::getTravelledDistance() const {
    // TODO
}
