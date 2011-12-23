/**
 (c) Lehrstuhl RA Universitaet Tuebingen

 @author: Bohlmann, Marks
 @date 2010

 @file PidSpeedController.cpp
 */

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

// Project
#include "PidSpeedController.h"
#include "MsgPrint.h"

///////////////////////////////////////////////////////////////////////////////
// IMPLEMENTATION
///////////////////////////////////////////////////////////////////////////////

/** Braking speed (if robot speed > 0) */
 const double BRAKE_SPEED = 5.0;
/** Allow aggressive braking if current speed is < AGGRESSIVE_BRAKE_THRESHOLD */
 //const double AGGRESSIVE_BRAKE_THESHOLD = 0.08;
 const double AGGRESSIVE_BRAKE_THESHOLD = 0.0;
/** Requested velocities smaller than ZERO_SPEED will be treated as zero speed request */
 const double ZERO_SPEED = 0.05;
/** Aggressive brake proportional parameter */
 const double BRAKE_KP = 0.07;
/** Time between zero speed commands in SLOWREVERSE mode [ms] */
 const int SLOWREVERSE_TIMEOUT = 500;
/** Barrier value between SLOWREVERSE and REVERSE state */
 const double REVERSE_SWITCH = 0.08;
/** Maximum speed servo value */
 const U16 SERVO_MAX = 3000;
/** Minimum speed servo value */
 const U16 SERVO_MIN = 700;
/** Zero servo value */
 const U16 SERVO_ZERO = 2250;

PidSpeedController::PidSpeedController( RamaxxConnection * conn,
                                        Actuators * actuators,
                                        Odometry * odo )
    : SpeedController( conn ), mSpeedFilter( 10 ), mLtSpeedFilter( 15 ) {
    // Init member
    mActuators = actuators;
    mOdo = odo;
    mState = STANDING;
    mTargetSpeed = 0.0f;
    mIntegralE = 0.0;
    mDeactivated = false;
    mOdoValid = false;
    mLastSpeedDiff = 0.0;
    mStall = false;
    mAllowEddyBrake = false;
    // Default calib
    mCalib.nullBackward = 2230;
    mCalib.nullForward = 2270;
    mCalib.speedScale = 100;
    mCalib.updateIntervalMs = 20;
    mCalib.kp = 0.5;
    mCalib.ki = 0.7;
    mCalib.kd = 0.05;
    // Restart timer and register to receive odometry data
    mUpdateTimer.restart();
    mSlowreverseTimer.restart();
    odo->addDataListener( this );
    // Development
    mLog.addColumn("v_target","v_target",false);
    mLog.addColumn("v_set","v_set",false);
    mLog.addColumn("v_is","v_is",false);
    mLog.addColumn("u","servo",false);
//    mLog.enable("/tmp/pid.log",true);
}

PidSpeedController::~PidSpeedController() {
    mOdo->removeDataListener( this );
} 


void PidSpeedController::setSpeed( float newTargetSpeed ) {

    mDeactivated = false;

    if ( fabs( newTargetSpeed ) < ZERO_SPEED ) {
        mState = STANDING;
        pidReset();
        mTargetSpeed = 0.0;
        return;
    }

    if ( mCalib.maxSpeed > ZERO_SPEED && fabs( newTargetSpeed ) > mCalib.maxSpeed ) {
        mTargetSpeed = newTargetSpeed > 0 ? mCalib.maxSpeed : -mCalib.maxSpeed;
    } else {
        mTargetSpeed = newTargetSpeed;
    }

    if ( mSpeedFilter.GetValue() >= 0.0) {
        // Current speed > 0
        if ( newTargetSpeed <= -ZERO_SPEED ) {
            if ( mSpeedFilter.GetValue() > 0 ) {
                mState = SWITCHBRAKE;
                pidReset();
            } else {
		mState = SLOWREVERSE;
            }
        } else {
            mState = FORWARD;
        }
    } else {
        // Current robot speed < 0
        if ( newTargetSpeed <= -ZERO_SPEED ) {
            mState = SLOWREVERSE; // Switches to state REVERSE if the robot is moving fast enough
        } else {
            mState = FORWARD;
            pidReset();
        }
    }
    mLog.addValue("v_target",mTargetSpeed );
    update();
}


void PidSpeedController::setCalibration( const SpeedCtrlCalibration &calib ) {
    mCalib = calib;
    pidReset();
    update();
}


void PidSpeedController::update( bool immediately ) {

    if ( mDeactivated || ( !immediately && mUpdateTimer.msElapsed() < mCalib.updateIntervalMs )) {
        return; // Update only every xxx msec, dont send any values if we are deactivated
    }

    // Check if the odometry reports valid data
    if ( !mOdoValid ) {
        // No valid speed values available
        setMotorSpeed( mTargetSpeed );
        return;
    }

    mLog.addValue("v_is",mSpeedFilter.GetValue());

    // Difference between requested and measured speed
    double speedDiff = mTargetSpeed - mSpeedFilter.GetValue();
    // New speed
    double speedSet = 0.0;
    // Current speed as reported by the odometry system
    double currentSpeed = mSpeedFilter.GetValue();
    if ( currentSpeed < 0 ) mAllowEddyBrake = false;
    // Update error integral
    if (fabs(currentSpeed)>0.2) {
        mIntegralE += speedDiff*mUpdateTimer.sElapsedDouble();
    }
    switch ( mState ) {
    case SWITCHBRAKE:
        // Switch status to REVERSE?
        if ( fabs( currentSpeed ) < AGGRESSIVE_BRAKE_THESHOLD ) {
            mSlowreverseTimer.restart();
            pidReset();
            speedSet = 0.0;
            mState = SLOWREVERSE;
            break;
        }
        // Braking
        if ( currentSpeed > AGGRESSIVE_BRAKE_THESHOLD && mAllowEddyBrake ) {
            speedSet = -BRAKE_SPEED;
        } else {
            // Aggressive
            speedSet = -BRAKE_KP*currentSpeed;
        }
        break;

    case SLOWREVERSE:
        if ( currentSpeed < -REVERSE_SWITCH ) {
            mState = REVERSE;
            // No break!
        } else if ( mSlowreverseTimer.msElapsed() > SLOWREVERSE_TIMEOUT ) {
            // Send zero value to switch to reverse mode
            speedSet = 0.0;
            mSlowreverseTimer.restart();
            break;
        } else {
            speedSet = regulate( mTargetSpeed, speedDiff, mLastSpeedDiff, mUpdateTimer.sElapsedDouble());
            break;
        }

    case REVERSE:
        if ( mLtSpeedFilter.GetValue() >= 0.0 ) {
            // Switch to slow reverse if we are standing (or moving forward)
            mState = SLOWREVERSE;
            speedSet = 0.0;
            mSlowreverseTimer.restart();
            break;
        }
        // No break!
    case FORWARD:
        speedSet = regulate( mTargetSpeed, speedDiff, mLastSpeedDiff, mUpdateTimer.sElapsedDouble());
        if ( speedSet > 0 && mState == REVERSE ) {
            // Dont allow values > 0 if we are moving backwards
            // since the controller switches to forward mode
            speedSet = 0;
        }
        break;

    case STANDING:
        if ( fabs( mLtSpeedFilter.GetValue()) < 0.15 ) {
            speedSet = 0.0;
        } else if ( mLtSpeedFilter.GetValue() > AGGRESSIVE_BRAKE_THESHOLD && mAllowEddyBrake ) {
            // Forward braking
            speedSet = -BRAKE_SPEED;
        } else if ( mLtSpeedFilter.GetValue() < -AGGRESSIVE_BRAKE_THESHOLD ) {
            speedSet = 0.0; // TODO Reverse braking
        } else {
            // Aggressive braking
            speedSet = -BRAKE_KP*currentSpeed;
        }
    }
    setMotorSpeed( speedSet );
    mLastSpeedDiff = speedDiff;
    if ( speedSet > 0 && currentSpeed > ZERO_SPEED ) mAllowEddyBrake = true;
    //DEBUGOUT << "Allow eddy: " << mAllowEddyBrake << endl;
    mUpdateTimer.restart();

    mLog.addValue("v_set",speedSet );
    mLog.writeLogLine();

}

void PidSpeedController::dataChanged( const DataChangeEvent &event ) {
    // We are observing only one data owner (odometry)
    switch ( event.eventType ) {
    // Odometry reports valid data?
    case DataChangeEvent::STATE:
        if ( !mOdo->hasValidData()) {
            if ( mOdoValid ) {
                //player: ERRORPRINT( "Odometry reports invalid data. Cannot control the speed of the robot." );
                mOdoValid = false;
                pidReset();
            } else {
                mOdoValid = true;
            }
        }
        break;

    // New odometry data available
    case DataChangeEvent::NEW_DATA:
        mOdoValid = true;
        mSpeedFilter.Update( mOdo->getVelocity());
        mLtSpeedFilter.Update( mOdo->getVelocity());
        break;
    }
}

double PidSpeedController::regulate(
        const double targetSpeed,
        const double speedDiff,
        const double lastSpeedDiff,
        const double timeDiff ) const {
    double speedSet = 0;
    speedSet = targetSpeed + mCalib.kp*speedDiff + mCalib.ki*mIntegralE;
    if ( timeDiff > 1E-1 ) {
        speedSet += ( speedDiff - lastSpeedDiff ) / timeDiff;
    }

    if (speedSet>1.5*targetSpeed) {
        speedSet=1.5*targetSpeed;
    }
    if (speedSet<0.5*targetSpeed) {
        speedSet=0.5*targetSpeed;
    }
    return speedSet;
}

void PidSpeedController::pidReset() {
    mIntegralE = 0;
    mLastSpeedDiff = 0;
}

void PidSpeedController::deactivate() {
    mDeactivated = true;
}

void PidSpeedController::setMotorSpeed( double speedMSec ) {

    U16 servoVal = SERVO_ZERO;
    if ( speedMSec > 0 ) { // forward
        servoVal= ((U16)(speedMSec * mCalib.speedScale)) + mCalib.nullForward;
        if ( servoVal > SERVO_MAX ) servoVal = SERVO_MAX;
    } else if ( speedMSec < 0 ) { // backward
        servoVal = ((U16)(speedMSec * mCalib.speedScale)) + mCalib.nullBackward;
        if ( servoVal < SERVO_MIN ) servoVal = SERVO_MIN;
    }

    mLog.addValue("u",servoVal/1000.0);
    mActuators->setSpeed(servoVal);
}
