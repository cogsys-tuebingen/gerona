/*
 * @file Ramaxx.cpp
 *
 * @date Jul 14, 2009 early 21st century
 * @author bohlmann, marks
 */

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

// C/C++
#include <iostream>
#include <string>
#include <iomanip>

// Workspace
#include <Misc.h>
#include <Strings.h>

// Project
#include "Ramaxx.h"
#include "PidSpeedController.h"
#include "MsgPrint.h"

// Eigen vector math
#include "Eigen/Core"

// Import most common Eigen types
using namespace Eigen;


///////////////////////////////////////////////////////////////////////////////
// DEFINITIONS
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
// IMPLEMENTATION
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace Ra;

Ramaxx* Ramaxx::mRamaxx;

void Ramaxx::initialize() {
    mRamaxx = new Ramaxx();
}

Ramaxx::Ramaxx()
    : mActuators( &mUsbConn ), mSensors( &mUsbConn ),
    mSteerMode( FRONT ), mSteerFront( &mActuators, &mSensors ),
    mSteerBack( &mActuators, &mSensors ),
    mRamaxxOdo( &mSensors, &mSteerFront, &mSteerBack ),
    mPtz( &mActuators, &mSensors), mRtz( &mActuators ),
      mBackSteerAvail( false ),mRawSteerServoMode(false), mCmdMode(Ra::AUTO), mAvrUi( &mUsbConn ),
    mAvrParams( &mUsbConn, &mSensors, &mActuators )
{
    mOdo = &mRamaxxOdo;
    mSpeed = NULL;
    setSpeedCtrlType( PID );
    gettimeofday( &mLastWdtReset, 0 );
    mDiagnosisTimer.restart();
    mDiagnosisInterval = 2500; // 2.5 sec first sensor diagnosis
}


Ramaxx::~Ramaxx() {
    close();
}


bool Ramaxx::open() {
    // Check if AVR32 is present
    if ( !mUsbConn.isAvr32Present()) {
        //DEBUGOUT << "Ramaxx: Cannot open AVR32, device NOT present." << endl;
        return false;   // Device not present
    }

    // Device is present, try to open it
    if ( !mUsbConn.openAvr32()) {
        ERRORPRINT( "Cannot open AVR32. Device is present." );
        mUsbConn.closeAvr32();
        return false;
    }
    INFOPRINT( "AVR32 opened" ); ;

    // Send setup request
    if ( !mUsbConn.sendSetupRequest()) {
        ERRORPRINT( "Ramaxx: Cannot send setup request to the avr32." );
        mUsbConn.closeAvr32();
        return false;
    }

    return true;
}

int Ramaxx::resetRobotWatchdog() {
    // Check if its necessary to reset the wdt
    timeval now;
    gettimeofday( &now, 0 );
    if ( Misc::getTimeDiff( &mLastWdtReset, &now ) > 0.15 ) {
        if ( mUsbConn.sendWdtResetRequest()) {
            gettimeofday( &mLastWdtReset, 0 );
            return 1;
        }
        else
            return -1;
    }
    return 0;
}

void Ramaxx::setSpeedCtrlType( SpeedCtrlType type ) {
    // Delete old speed controller
    float requestedSpeed = 0;
    if ( mSpeed != NULL ) {
        requestedSpeed = mSpeed->getSpeed();
        delete mSpeed;
    }

    // Create new speed controller object
    switch ( type ) {
    case SIMPLE:
        mSpeed = new SimpleSpeedController( &mUsbConn, &mActuators, mOdo );
        break;
    case PID:
        mSpeed = new PidSpeedController( &mUsbConn, &mActuators, mOdo );
        break;
    case AVR32:
        mSpeed = new Avr32SpeedController( &mUsbConn );
        break;
    }

    // Set requested speed
    mSpeed->setSpeed( requestedSpeed );
}

void Ramaxx::addQuMsgHandler( U8 msgType, QuMsgHandler &handler ) {
    mUsbConn.addQuMsgHandler( msgType, handler );
}

void Ramaxx::addQuMsgHandler( U8 msgTypes[], size_t length, QuMsgHandler &handler ) {
    mUsbConn.addQuMsgHandler( msgTypes, length, handler );
}

void Ramaxx::addQuMsgHandler( QuMsgHandler &handler ) {
    mUsbConn.addQuMsgHandler( handler );
}

void Ramaxx::displayText( const string& text, int lineNr ) {
    QuMessage msg;
    string s=text.substr( 0,MT_STRING_MAX_LEN );
    msg.type = MT_STRING;
    msg.length = s.length()+1;
    msg.data.resize(msg.length);
    strcpy((char *)&(msg.data[0]),s.c_str());
    queueMsg(msg);
}

void Ramaxx::beep( const int *beeps ) {
    mActuators.beep( beeps );
}

void Ramaxx::setFrontSteerServoConfig(const SteerServoConfig &front) {
    mSteerFront.setConfig( front );
}

void Ramaxx::setRearSteerServoConfig(const SteerServoConfig &rear) {
    mSteerBack.setConfig( rear );
}

void Ramaxx::adjustSteerFront( int delta ) {
    mSteerFront.adjustServoZero( delta );
}

void Ramaxx::adjustSteerBack( int delta ) {
    mSteerBack.adjustServoZero( delta );
}

void Ramaxx::setBackSteerAvailable( bool available ) {
    mBackSteerAvail = available;
    if ( !mBackSteerAvail ) {
        mSteerBack.setAngleDeg( 0 );
    }
}

void Ramaxx::setSteerMode( const RobotSteerMode &mode ) {
    if ( mBackSteerAvail ) {
        mSteerMode = mode;
        //DEBUGOUT << "Ramaxx: Set steer mode to " << mode << endl;
    } else {
        mSteerMode = FRONT; // Dont change steer mode if the rear axis is not steerable
        if ( mode != FRONT ) {
            ERRORPRINT( "Ramaxx: Cannot change steer mode. Rear axis is not steerable, steer mode is \"front\"." );
        }
    }
}

void Ramaxx::setSteerDegrees( float angle ) {
    mRawSteerServoMode=false;
    if ( mSteerMode == FRONT ) {
        mSteerFront.setAngleDeg( angle );
        mSteerBack.setAngleDeg( 0 );
    } else if ( mSteerMode == REAR ) {
        mSteerFront.setAngleDeg( 0 ),
        mSteerBack.setAngleDeg( -angle );
    } else if ( mSteerMode == BOTH ) {
        mSteerFront.setAngleDeg( angle ),
        mSteerBack.setAngleDeg( -angle );
    } else if ( mSteerMode == PARALLEL ) {
        mSteerFront.setAngleDeg( angle );
        mSteerBack.setAngleDeg( angle );
    }
}

float Ramaxx::getSteerDegrees() {
    if ( mSteerMode == REAR )
        return -mSteerBack.getAngleDeg();
    else
        return mSteerFront.getAngleDeg();
}

bool Ramaxx::processRobotMsgs() {
    if ( !mUsbConn.processRobotMsgs()) {
            ERRORPRINT( "Ramaxx: ERROR processing robot messages." );
            mUsbConn.closeAvr32();
            return false;
        }
    return true;
}

int Ramaxx::close() {
// TODO: Sending shutdown request hangs.
//    mUsbConn.sendShutdownRequest();
    return EOK;
}

void Ramaxx::update () {
    // Reset watchdog
    resetRobotWatchdog();
    // Update speed controller
    mSpeed->update( false );
    // Update steer controller
    if (!mRawSteerServoMode) {
        mSteerFront.update();
        mSteerBack.update();
    }
    // Update ptz & rtz controller
    mPtz.update();
    mRtz.update();
    // Update Avr user interface
    mAvrUi.update();
    // Run sensor diagnosis?
    if ( mDiagnosisTimer.msElapsed() > mDiagnosisInterval ) {
        mSensors.runDiagnosis( mDiagnosisTimer.msElapsed());
        mDiagnosisTimer.restart();
        mDiagnosisInterval = 250; // Faster error detection if "booted"
    }
}

void Ramaxx::setSteerFrontDegrees( float angle ) {
    mRawSteerServoMode=false;
    mSteerFront.setAngleDeg( angle );
}

void Ramaxx::setSteerBackDegrees( float angle ) {
    mRawSteerServoMode=false;
    mSteerBack.setAngleDeg( angle );
}


void Ramaxx::getOdoPos( Vector3d& pos ) {
    mOdo->getPosition( pos );
}

void Ramaxx::getPlainOdoPos( Vector3d& pos ) {
    mOdo->getRawPosition( pos );
}


void Ramaxx::getOdoVelocity( Vector3d& vel ) {

    mOdo->getVelocity( vel);
}

void Ramaxx::setOdoPos( const Vector3d& pos ) {
    mOdo->setPosition( pos );
}

bool Ramaxx::hasEncoderLeft() {
    return mSensors.getRearWheelEncoder()->isLeftEnabled();
}

bool Ramaxx::hasEncoderRight() {
    return mSensors.getRearWheelEncoder()->isRightEnabled();
}

Actuators* Ramaxx::getActuators() {
	return &mActuators;
}

void Ramaxx::getKinematicState( float& speed, float& steerFront, float& steerRear ) {
    speed = (float)mOdo->getVelocity();
    steerFront = (float)mSteerFront.getAngleDeg();
    steerRear = (float)mSteerBack.getAngleDeg();
}

void Ramaxx::setSpeed( float speed ) {
    mSpeed->setSpeed( speed );
}

void Ramaxx::deactivateSpeedController() {
    mSpeed->deactivate();
}

void Ramaxx::setSpeedCtrlCalibration( const SpeedCtrlCalibration &calib ) {
    mSpeed->setCalibration( calib );
}

void Ramaxx::setPtzCalibration( const PtzCalibration &calib ) {
    mPtz.setCalibration( calib );
}

void Ramaxx::setPanRad( float angle ) {
    mPtz.setPanRad( angle );
}

void Ramaxx::setTiltRad( float angle ) {
    mPtz.setTiltRad( angle );
}

float Ramaxx::getPanRad() const {
    return mPtz.getPanRad();
}

float Ramaxx::getTiltRad() const {
    return mPtz.getTiltRad();
}

void Ramaxx::adjustPan( int delta ) {
    mPtz.adjustPanServoZero( delta );
}

void Ramaxx::adjustTilt( int delta ) {
    mPtz.adjustTiltServoZero( delta );
}

void Ramaxx::setCmdMode( const RobotCmdMode &mode ) {
    if ( mode != mCmdMode ) {
        switch ( mode ) {
        case AUTO:
            INFOPRINT( "ATTENTION PLEASE: Command mode set to AUTOMATIC. Take cover." );
            { int beeps[] = { 50, 50, 0 };
            beep( beeps ); }
            break;

        case MANUAL:
            INFOPRINT( "Command mode set to MANUAL." );
            break;

        case MANUAL_EXCEPT_PTZ:
            INFOPRINT( "Command mode set to MANUAL except PTZ." );
            break;
        }
        mCmdMode = mode;
    }
}

void Ramaxx::getLogState( bool& isLogging, int& logSuffix ) {
    mAvrUi.getLogRequest( isLogging, logSuffix );
}


void Ramaxx::setLogInfo( bool isLogging, U16 fileNumber, U32 size ) {
    mAvrUi.setLogInfo( isLogging, fileNumber, size );
}
