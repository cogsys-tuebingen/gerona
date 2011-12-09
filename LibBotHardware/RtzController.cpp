/*
 * PtzController.cpp
 *
 *  Created on: Nov, 2011
 *      Author: marks
 */

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

// Project
#include "RtzController.h"

///////////////////////////////////////////////////////////////////////////////
// IMPLEMENTATION
///////////////////////////////////////////////////////////////////////////////

//// class RtzController //////////////////////////////////////////////////////

RtzController::RtzController(Actuators *act) :
    mRollId( Actuators::ACTUATOR_LASER_ROLL ),
    mTiltId( Actuators::ACTUATOR_LASER_TILT ),
    mActuators( act ),
    mRollRequest( 0.0 ),
    mTiltRequest( 0.0 ),
    mNewRequest( false )
{
    mCalib.rollDegToServo = 190.8959;
    mCalib.tiltDegToServo = 190.8959;
    mUpdateStopwatch.restart();

    resetPosition();
    mNewRequest = true; // Force a initial position update
}

void RtzController::update() {
    if ( mUpdateStopwatch.msElapsed() < 15 )
        return; // Nothing to do. Update only every 15 msec

    if ( true /* mNewRequest */ ) { // TODO What happens is we are not connected to the Avr32?!?
        mActuators->setActuator( mRollId, mActuators->getActuatorZero( mRollId )
                                + mCalib.rollDegToServo*180.0*mRollRequest/M_PI);
        mActuators->setActuator( mTiltId, mActuators->getActuatorZero( mTiltId )
                                + mCalib.tiltDegToServo*180.0*mTiltRequest/M_PI);
        mUpdateStopwatch.restart();
        mNewRequest = false;
    }
}

void RtzController::setRollTiltDegree( const double &roll, const double &tilt ) {
    setRollRad( M_PI*roll/180.0 );
    setTiltRad( M_PI*tilt/180.0 );
}

void RtzController::setRollTiltRad( const double &roll, const double &tilt ) {
    setRollRad( roll );
    setTiltRad( tilt );
}

void RtzController::setRollRad(const double &roll) {
    if ( roll != mRollRequest ) {
        mRollRequest = roll;
        mNewRequest = true;
    }
}

void RtzController::setTiltRad(const double &tilt) {
    if ( tilt != mTiltRequest ) {
        mTiltRequest = tilt;
        mNewRequest = true;
    }
}

void RtzController::resetPosition() {
    setRollTiltRad( 0.0, 0.0 );
}

void RtzController::setCalibration(const RtzCalibration &calib) {
    mCalib = calib;
    setRollTiltRad( mRollRequest, mTiltRequest );
    mNewRequest = true; // Force an update here
}
