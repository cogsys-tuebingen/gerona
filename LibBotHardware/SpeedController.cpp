/*
 * SpeedController.cpp
 *
 *  Created on: Nov 19, 2009
 *      Author: marks
 */

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

// C/C++
//#include <math.h>
#include <iomanip>

// Workspace
#include <Misc.h>

// Project
#include "SpeedController.h"
#include "Ramaxx.h"
#include "QuMessage.h"

#define fixpoint(width, decimals) fixed << right << setw(width) << setprecision(decimals)

///////////////////////////////////////////////////////////////////////////////
// IMPLEMENTATION
///////////////////////////////////////////////////////////////////////////////

//// class SpeedController ////////////////////////////////////////////////////

SpeedController::SpeedController( RamaxxConnection *conn )
    : mAvr32Ctrl( false ), mConn( conn ) {
    mConn->addQuMsgHandler( MT_SPEEDCTRL, *this );

}

SpeedController::~SpeedController() {
    mConn->removeQuMsgHandler( MT_SPEEDCTRL );
}

void SpeedController::setAvr32SpeedCtrlEnabled( bool arg ) {
    mAvr32Ctrl = arg;
    if ( mConn->isOpen())
        connectionEstablished(); // Send enable/disable request
}

void SpeedController::connectionEstablished() {
    QuMessage msg;
    msg.type = MT_SPEEDCTRL;

    if ( mAvr32Ctrl ) {
        // Enable command
        msg.length = 13;
        msg.data.resize( 13 );
        msg.data[0] = 1;

        // P
        S32 val = mCalib.kp;
        msg.data[1] = (val >> 24) & 0xFF;
        msg.data[2] = (val >> 16) & 0xFF;
        msg.data[3] = (val >> 8) & 0xFF;
        msg.data[4] = val & 0xFF;

        // I
        val = mCalib.ki;
        msg.data[5] = (val >> 24) & 0xFF;
        msg.data[6] = (val >> 16) & 0xFF;
        msg.data[7] = (val >> 8) & 0xFF;
        msg.data[8] = val & 0xFF;

        // Brake scale
        val = mCalib.brakeScale;
        if ( val < 1 ) val = 1;
        msg.data[9] = (val >> 24) & 0xFF;
        msg.data[10] = (val >> 16) & 0xFF;
        msg.data[11] = (val >> 8) & 0xFF;
        msg.data[12] = val & 0xFF;

    } else {
        // Disable command
        msg.length = 1;
        msg.data.resize( 1 );
        msg.data[0] = 0;
    }
}

//// class Avr32SpeedController ///////////////////////////////////////////////

Avr32SpeedController::Avr32SpeedController( RamaxxConnection * conn )
  : SpeedController( conn ), mTargetSpeed( 0 ), mConn( conn ) {
    mUpdateTimer.restart();
    setAvr32SpeedCtrlEnabled( true );

    // Initial config
    mCalib.kp = 175;
    mCalib.ki = 25;
    mCalib.brakeScale = 1;
}

void Avr32SpeedController::setSpeed( float speed ) {
    mTargetSpeed = speed;
    update( false );
}

void Avr32SpeedController::update( bool immediately ) {
    // Check time since last update
    if ( !immediately && mUpdateTimer.msElapsed() < 25 ) {
        return;
    }
    mUpdateTimer.restart();

    // Send requested speed to the microcontroller
    S32 value = (S32)(1000*mTargetSpeed);
    QuMessage msg;
    msg.type = MT_SPEED;
    msg.length = 4;
    msg.data.resize( msg.length );
    msg.data[0] = ( value >> 24 ) & 0xFF;
    msg.data[1] = ( value >> 16 ) & 0xFF;
    msg.data[2] = ( value >> 8 ) & 0xFF;
    msg.data[3] = value & 0xFF;
    mConn->queueMsg( msg );
}

void Avr32SpeedController::deactivate() {
    // TODO Implement
}

//// class SimpleSpeedController //////////////////////////////////////////////

SimpleSpeedController::SimpleSpeedController( RamaxxConnection * conn,
                                             Actuators * actuators, Odometry * odo )
:
    SpeedController( conn ),
    mActuators(actuators),
    mNullForward(2250),
    mNullBackward(2250),
    mSpeedScale(100),
    mTargetSpeed(0),
    mActualSpeed(0),
    mSpeed(0),
    mBackwards(false),
    mOdo(odo),
    mStall(0),
    mActive(true),
    mWheelEncoder(true),
    mCycler(0),
    mControl(1.0f)
{
    gettimeofday(&mLastSend, 0);
    if ( odo != NULL ) {
        mOdo->addDataListener( this );
    }
    setAvr32SpeedCtrlEnabled( false );

    // Set initial calib values
    mCalib.nullForward = mNullForward;
    mCalib.nullBackward = mNullBackward;
    mCalib.speedScale = mSpeedScale;
}

void SimpleSpeedController::setCalibration( const SpeedCtrlCalibration &calib ) {
    mNullForward = calib.nullForward;
    mNullBackward = calib.nullBackward;
    mSpeedScale = calib.speedScale;
}

void SimpleSpeedController::setServoValue( U16 value ) {
    mActuators->setSpeed( value );
}

void SimpleSpeedController::setSpeed( float speed ) {
    mTargetSpeed = speed;

    // reactivate
    mActive = true;

    // update controller immediately
    update(true);
}

float bangbangmult(float targetSpeed, float actualSpeed) {
	return fabs(targetSpeed - actualSpeed) < 0.01f ? 1.0f : (fabs(targetSpeed) > fabs(actualSpeed) ? 1.1f : 0.9f);
}

void SimpleSpeedController::deactivate() {
	mActive = false;
}

void SimpleSpeedController::update(bool immediately) {
	// only act when active
	if (!mActive)
		return;

	// update interval
#define INTERVAL 0.3

    float servo = thrustToServo(mTargetSpeed * mControl);

    timeval now;
    gettimeofday(&now, 0);
    double dt = Misc::getTimeDiff( &mLastSend, &now );

	// check for stall and presence of wheel encoder
    if (dt > INTERVAL) {
    	if (mWheelEncoder) {
			if ( (fabs(mTargetSpeed) > 0.1f)
			  && (fabs(mActualSpeed) < 0.1f)
			  && (mControl > 2.5f) ) {
				++mStall;

				if (mStall > 1) {
					if ( (mStall > 10) && (fabs(mActualSpeed) < 0.00001f) ) {
						cout << "SpeedController: No working wheel encoder!" << endl;
						mWheelEncoder = false;
						mControl = 1.0f;
					}
					else {
						cout << "SpeedController: STALL!" << endl;
					}
				}
			}
			else {
				mStall = 0;
			}
    	}
    	else {
			cout << "SpeedController: No working wheel encoder!" << endl;
    	}
    }

    if (dt > INTERVAL) {
    	// control speed
    	// control is a factor that increases as soon as friction increases
    	// if the robot is driving on the ground it should be around 1
    	// if the wheels don't touch the ground it is usually about 0.05
    	if (!mStall && mWheelEncoder)
    		mControl *= bangbangmult(mTargetSpeed, mActualSpeed);

    	if (mTargetSpeed < 0)
    		servo = thrustToServo(mTargetSpeed * mControl * 1.125f); // correction factor
    	else
    		servo = thrustToServo(mTargetSpeed * mControl);

/*    	cout << "SpeedController: servo: " << fixpoint(4,3) << mActuators->getSpeed()
    		 << ", targetS: " << fixpoint(7,3) << mTargetSpeed
    		 << ", actualS: " << fixpoint(7,3) << mActualSpeed
             << ", control: " << fixpoint(7,3) << control << endl;
        cout << fixpoint(4,3) << mActuators->getSpeed()
    		 << fixpoint(7,3) << mTargetSpeed
    		 << fixpoint(7,3) << mActualSpeed
             << fixpoint(7,3) << mControl << endl;*/
    }

    /**
     * The backward problem:
     * If the robot is driving forward, the EVX interprets backward servo commands
     * as brake commands. The robot is actually braking until it stops. The EVX
     * then suppresses the backward commands to prevent the robot from driving
     * backwards. If driving backwards is desired, a zero command has to be sent,
     * followed by the backward commands. This might be comfortable for manual
     * steering, but leaves the programmer with the challenge to find a method
     * that wraps around the EVX, so that a client can send a backward command
     * which is reliably executed by the EVX. The current solution to this is to send
     * a zero command every 140 msecs for 20 msecs. If you find a better workaround
     * for the black box called EVX, feel free to implement.
     */
    if (mTargetSpeed < 0) { // backwards
    	if (dt > 0.02) {
        	if (mCycler > 7) {
                setServoValue(mNullBackward+2); // zero command
        		mCycler = 0;
        	}
        	else {
        		setServoValue(servo); // desired backward command
        		++mCycler;
        	}
        }
    }
    else { // forward
        if (immediately || (dt > INTERVAL)) {
        	setServoValue(servo);
        	mCycler = 0;
        }
    }

    // reset time
    if (dt > INTERVAL)
    	gettimeofday(&mLastSend, 0);
}

void SimpleSpeedController::dataChanged( const DataChangeEvent &event ) {
    // TODO What happens if there is no valid odometry data?
    if (event.owner == mOdo && event.eventType == DataChangeEvent::NEW_DATA
        && mOdo->hasValidData()) {
        mActualSpeed = mOdo->getVelocity();
    }
}

double SimpleSpeedController::servoToThrust( int servoVal ) const {
    if ( servoVal < mNullBackward )
        return ((double)( servoVal - mNullBackward )) / mSpeedScale;

    if ( servoVal > mNullForward )
        return ((double)( servoVal - mNullForward )) / mSpeedScale;

    return 0.0f;
}

U16 SimpleSpeedController::thrustToServo( float thrust ) const {
    if ( thrust > 1e-6) {    // forward
        return ((U16)(thrust * mSpeedScale)) + mNullForward;
    } else if ( thrust < -1e-6 )  {  // backward
        return ((U16)(thrust * mSpeedScale)) + mNullBackward;
    }
    return 2250; // Servo mid
}






