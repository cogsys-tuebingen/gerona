/*
 * PtzController.cpp
 *
 *  Created on: Feb 20, 2010
 *      Author: marks
 */

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

// C/C++
#include <math.h>
#include <iostream>

// Project
#include "PtzController.h"

///////////////////////////////////////////////////////////////////////////////
// DEFINITIONS
///////////////////////////////////////////////////////////////////////////////

// Limits
const float     PAN_MAX_ANGLE = M_PI/2;
const float     PAN_MIN_ANGLE = -M_PI/2;
const float     TILT_MAX_ANGLE = M_PI/3;
const float     TILT_MIN_ANGLE = -M_PI/3;

///////////////////////////////////////////////////////////////////////////////
// IMPLEMENTATION
///////////////////////////////////////////////////////////////////////////////

//// class PtzController //////////////////////////////////////////////////////

PtzController::PtzController( Actuators * act, RobotSensors * sensors ) {
    mPanId = Actuators::ACTUATOR_CAM_PAN;
    mTiltId = Actuators::ACTUATOR_CAM_TILT;
    mFixedPan = mFixedTilt = false;
    mActuators = act;
    mSensors = sensors;
    mUpdateStopwatch.restart();
    setPanRad( 0.0f );
    setTiltRad( 0.0f );
    setValidData( true ); // Always valid data

    /*if ( sensors->hasPtzVoltage()) {
        sensors->getPanTiltVoltage()->addDataListener( this );
    }*/
}

void PtzController::update() {
    if ( mUpdateStopwatch.msElapsed() >= 20 ) {
        bool fireUpdate = false;        
        // Send pan/tilt servo values
        mUpdateStopwatch.restart();
        U16 servoValue;
        if ( !mFixedPan ) {
            servoValue = ((S32)( mPanRequest*180.0*mCalibration.panDegToServo/M_PI))
                         + mActuators->getActuatorZero( mPanId );
            fireUpdate = servoValue != mActuators->getActuator( mPanId );
            mActuators->setActuator( mPanId, servoValue );
        }
        if ( !mFixedTilt ) {
            servoValue = ((S32)( mTiltRequest*180.0*mCalibration.tiltDegToServo/M_PI))
                         + mActuators->getActuatorZero( mTiltId );
            fireUpdate = fireUpdate || servoValue != mActuators->getActuator( mTiltId );
            mActuators->setActuator( mTiltId, servoValue );
        }
        if ( fireUpdate )
            fireDataChanged();
    }
}

void PtzController::setPanTiltRad( const float &pan, const float &tilt ) {
    setPanRad( pan );
    setTiltRad( tilt );
}

void PtzController::setPanTiltDegree( const float &pan, const float &tilt ) {
    setPanRad( pan * M_PI / 180.0f );
    setTiltRad( tilt * M_PI / 180.0f );
}

void PtzController::setFixedPan( const float &pan ) {
    mFixedPan = true;
    mPan = pan;
    setPanRad( pan );
    fireDataChanged();
}

void PtzController::setFixedTilt( const float &tilt ) {
    mFixedTilt = true;
    mTilt = tilt;
    setTiltRad( tilt );
    fireDataChanged();
}

void PtzController::setPanRad( const float &pan ) {
    if ( !mFixedPan ) {
        mPanRequest = pan;

        // Limit angle
        if ( mPanRequest < PAN_MIN_ANGLE) mPanRequest = PAN_MIN_ANGLE;
        if ( mPanRequest > PAN_MAX_ANGLE) mPanRequest = PAN_MAX_ANGLE;
    }
}

void PtzController::setTiltRad( const float &tilt ) {
    if ( !mFixedTilt ) {
        mTiltRequest = tilt;

        // Limit angle
        if (mTiltRequest < TILT_MIN_ANGLE) mTiltRequest = TILT_MIN_ANGLE;
        if (mTiltRequest > TILT_MAX_ANGLE) mTiltRequest = TILT_MAX_ANGLE;
    }
}

float PtzController::getPanRad() const {
    // Check if the pan angle is fixed or if the ADC has valid data
    if ( mFixedPan || ( mSensors->hasPtzVoltage()
              && mSensors->getPanTiltVoltage()->hasValidData())) {
        return mPan;
    } else {
        // ADC does not have valid data
        return ((float)( mActuators->getActuator( mPanId ) - mActuators->getActuatorZero( mPanId )))
                / M_PI*mCalibration.panDegToServo/180.0;
    }
}

float PtzController::getTiltRad() const {
    // Check if tilt angle is fixed or if the ADC has valid data
    if ( mFixedTilt || ( mSensors->hasPtzVoltage()
              && mSensors->getPanTiltVoltage()->hasValidData())) {
        return mTilt;
    } else {
        return ((float)( mActuators->getActuator( mTiltId ) - mActuators->getActuatorZero( mTiltId )))
                / M_PI*mCalibration.tiltDegToServo/180.0;
    }
}

void PtzController::setCalibration( const PtzCalibration &calib ) {
    mCalibration = calib;
    setPanTiltRad( mPanRequest, mTiltRequest );
    fireDataChanged();
}

void PtzController::adjustPanServoZero( int delta ) {
    ActuatorConfig conf = mActuators->getActuatorConfig( mPanId );
    conf.zero += delta;
    mActuators->setActuatorConfig( mPanId, conf );
    setPanRad( 0 );
}

void PtzController::adjustTiltServoZero( int delta ) {
    ActuatorConfig conf = mActuators->getActuatorConfig( mTiltId );
    conf.zero += delta;
    mActuators->setActuatorConfig( mTiltId, conf );
    setTiltRad( 0 );
}

void PtzController::resetPosition() {
    setPanTiltRad( 0, 0 );
}

// TODO: refactor this!!!
void PtzController::dataChanged( const DataChangeEvent &event ) {
    // We are observing only one data owner
    if ( event.eventType == DataChangeEvent::NEW_DATA
         && mSensors->hasPtzVoltage() && mSensors->getPanTiltVoltage()->hasValidData()) {
        // Use the measured voltages
        // Channel 0 is pan, channel 1 tilt
        /*float volts = mSensors->getPanTiltVoltage()->getChannel( 0 );
        if ( !mFixedPan )
            mPan = ( volts - mCalibration.panVoltageOffset ) * mCalibration.panVoltageToAngle;
        volts = mSensors->getPanTiltVoltage()->getChannel( 1 );
        if ( !mFixedTilt )
            mTilt = ( volts - mCalibration.tiltVoltageOffset ) * mCalibration.tiltVoltageToAngle;
        fireDataChanged();
        float pan = mActuators->getActuator( mPanId );
        if ( !mFixedPan )
            mPan = ( pan - mCalibration.panServoOffset ) / mCalibration.panAngleToServo;
        float tilt = mActuators->getActuator( mTiltId );
        if ( !mFixedTilt )
            mTilt = ( tilt - mCalibration.tiltServoOffset ) / mCalibration.tiltAngleToServo;

        fireDataChanged();*/
    }
}
