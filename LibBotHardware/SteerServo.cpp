/*
 * SteerServo.cpp
 *
 *  Created on: Nov 19, 2009
 *      Author: marks
 */

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

// C/C++
#include <math.h>
#include <time.h>
#include <iostream>

// Workspace
#include "Misc.h"
#include "Global.h"

// Project
#include "MsgPrint.h"
#include "SteerServo.h"

///////////////////////////////////////////////////////////////////////////////
// IMPLEMENTATION
///////////////////////////////////////////////////////////////////////////////

//// class SteerServo /////////////////////////////////////////////////////////////////

SteerServo::SteerServo( Actuators * actuators, RobotSensors * sensors) :
        mActuators( actuators ),
        mAngleDeg( 0.0f ),
        mTargetAngleDeg( 0.0f ),
        mVoltage( NULL ),
        mHallOk( false )
{
    // Initialize members
    mUpdateTimer.restart();
    mLastCmdTimer.restart();

    // Get the ADC chip that measures the steering angle
    mVoltage = NULL;
    if ( sensors->hasSteerVoltage()) {
        mVoltage = sensors->getSteerVoltage();
        mVoltage->addDataListener( this );
    } else {
        // Inform user?
    }
}

SteerServo::~SteerServo() {
    if ( mVoltage != NULL ) {
        mVoltage->removeDataListener( this );
    }
}

void SteerServo::setAngleDeg( float angle ) {
    if ( !mConfig.enableCtrl || !mHallOk ) {
        // TODO Check
        //setValue( angleToServo( angle ));
        mTargetAngleDeg = angle;
    } else {
        mTargetAngleDeg = angle;
        float rad = angle*M_PI/180.0;
        mCtrl.setTargetAngle( rad );
        setValue( mCtrl.getStaticServoValue( rad ));
    }
}

float SteerServo::getAngleDeg() {
    // Check if there is a voltage sensor measuring the servo position.
    // If not use the known servo speed to compute the current angle
    if ( mHallOk && mConfig.useHall ) {
        return mAngleDeg;
    } else {
        // TODO Implement!!!
        /*if ( mTargetAngleDeg != mAngleDeg ) {
            float step = mLastCmdTimer.sElapsed()*mServoSpeed;
            mLastStepTimer.restart();
            if ( fabs( mAngleDeg - mTargetAngleDeg ) < step ) {
               mAngleDeg = mTargetAngleDeg;
            } else {
                if ( mAngleDeg < mTargetAngleDeg ) {
                    mAngleDeg += step;
                } else {
                    mAngleDeg -= step;
                }
            }
        }*/
        return mTargetAngleDeg;
    }
}

void SteerServo::update() {
    if ( mUpdateTimer.msElapsed() >= mConfig.updateIntervalMs ) {
        S32 servoVal = getServoZero();

        // Use steer angle controller?
        if ( mConfig.enableCtrl && mHallOk ) {
            mCtrl.execute( mUpdateTimer.msElapsed()/1000.0, mAngleDeg*M_PI/180.0, servoVal );
        } else {
            servoVal = angleToServo( mTargetAngleDeg );
        }

        // Send value
        setValue( servoVal );
        mUpdateTimer.restart();
    }
}

void SteerServo::adjustServoZero( int delta ) {
    setServoZero( getServoZero() + delta );
}

void SteerServo::setConfig( const SteerServoConfig &config ) {
    mConfig = config;
    mCtrl.setPiParameters( mConfig.ctrlKp, mConfig.ctrlKi );

    // Check hall parameters
    if ( mConfig.useHall && (mConfig.hallCoefficients1.size() < 1 || mConfig.hallCoefficients2.size() < 1 )) {
        ERRORPRINT( "Cannot use hall sensors. Calibration data incomplete (coefficient array size < 1)" );
        mConfig.useHall = false;
    }

    if ( !mConfig.useHall && mConfig.enableCtrl ) {
        ERRORPRINT( "Cannot enable steer angle control without hall sensors!" );
        mConfig.enableCtrl = false;
    }
}


void SteerServo::dataChanged( const DataChangeEvent &event ) {
    // We are observing only one data owner (steer angle ADC)
    if ( event.eventType == DataChangeEvent::NEW_DATA && mConfig.useHall && mVoltage->hasValidData() ) {
        // Calculate steer angle using the output of the hall sensors
        double v1, v2;
        getHallVoltages( mVoltage, v1, v2 );
        mAngleDeg = 180.0*calcHallAngle( v1, v2 )/M_PI;
        //DEBUGOUT << "Calulated: " << mAngleDeg << " Volt: " << v1 << " " << v2 << endl;
        mHallOk = true;
    } else {
        // Don't use hall sensors
        mHallOk = false;
    }
}

double SteerServo::calcHallAngle( const double v1, const double v2 ) const {

    // Calculate angle of hall sensor #1 and #2

    double angle1 = 0;
    for ( size_t i = 1; i < mConfig.hallCoefficients1.size(); ++i )
        angle1 += mConfig.hallCoefficients1[i]*pow( v1 + mConfig.hallCoefficients1[0], i - 1 );
    double angle2 = 0;
    for ( size_t i = 1; i < mConfig.hallCoefficients2.size(); ++i )
        angle2 += mConfig.hallCoefficients2[i]*pow( v2 + mConfig.hallCoefficients2[0], i - 1 );


    // Use #1, #2 or the average of both?
    // TODO 1.9 WTF ?!?
    if ( v1 < 1.9 )
        return angle1; // Use #1
    if ( v2 < 1.9 )
        return angle2; // USe #2

    return 0.5*(angle1 + angle2); // Use average value

    // thrain front with tractor tires
    //double angle1= 0.426079+-0.224702*pow((x1+-1.026541),2)+-0.136907*pow((x1+-1.026541),4);
    //double angle2 = -0.404312+0.532972*pow((x2+-1.016608),2)+-0.280574*pow((x2+-1.016608),4)+0.148680*pow((x2+-1.016608),6);

    // thrain rear with tractor tires
    //double angle1=-0.362759+0.311860*pow((x1+-0.760034),2)+-0.118582*pow((x1+-0.760034),4)+0.040773*pow((x1+-0.760034),6)+0.003674*pow((x1+-0.760034),8);
    //double angle2=0.412166+-0.126369*pow((x2+0.382263),2)+0.064031*pow((x2+0.382263),4)+-0.023409*pow((x2+0.382263),6)+0.003665*pow((x2+0.382263),8)+-0.000216*pow((x2+0.382263),10);

    // gloin front
    //double angle1=0.430669+-0.220365*pow((x1+-0.988286),2)+-0.125045*pow((x1+-0.988286),4);
    //double angle2=-0.532563+0.312018*pow((x2+-0.485655),2)+-0.105408*pow((x2+-0.485655),4)+0.029200*pow((x2+-0.485655),6);


    // gloin rear
    //double angle1=-0.378514+0.193966*pow((x1+-0.616192),2)+0.031672*pow((x1+-0.616192),4)+-0.041144*pow((x1+-0.616192),6)+0.013846*pow((x1+-0.616192),8);
    //double angle2=0.381402+-0.155638*pow((x2+0.026753),2)+0.093779*pow((x2+0.026753),4)+-0.046377*pow((x2+0.026753),6)+0.009954*pow((x2+0.026753),8)+-0.000804*pow((x2+0.026753),10);
}

//// class FrontSteerServo ////////////////////////////////////////////////////

FrontSteerServo::FrontSteerServo( Actuators *actuators, RobotSensors *sensors ) :
        SteerServo( actuators, sensors ) {
  mCtrl.setPosition( 0 );
}

S32 FrontSteerServo::angleToServo( const float angleDeg ) const {
    return angleDeg * mConfig.degToServo + getServoZero();
}

void FrontSteerServo::setValue( const S32 value ) {
    mActuators->setSteerFront( value );
}

void FrontSteerServo::setServoZero( const S32 zeroValue ) {
    ActuatorConfig config = mActuators->getConfig( Actuators::ACTUATOR_STEER_FRONT );
    config.zero = zeroValue;
    mActuators->setConfig( Actuators::ACTUATOR_STEER_FRONT, config );
}

S32 FrontSteerServo::getServoZero() const {
    return mActuators->getZero( Actuators::ACTUATOR_STEER_FRONT );
}

void FrontSteerServo::getHallVoltages( Voltage *sensor, double &outVoltage1, double &outVoltage2 ) {
    outVoltage1 = sensor->getChannel( 0 );
    outVoltage2 = sensor->getChannel( 1 );
}

//// class BackSteerServo /////////////////////////////////////////////////////

BackSteerServo::BackSteerServo( Actuators *actuators, RobotSensors *sensors ) :
        SteerServo( actuators, sensors ) {
  mCtrl.setPosition( 0 );
}

S32 BackSteerServo::angleToServo( const float angleDeg ) const {
    return angleDeg * mConfig.degToServo + getServoZero();
}

void BackSteerServo::setValue( const S32 value ) {
    mActuators->setSteerBack( value );
}

void BackSteerServo::setServoZero( const S32 zeroValue ) {
    ActuatorConfig config = mActuators->getConfig( Actuators::ACTUATOR_STEER_REAR );
    config.zero = zeroValue;
    mActuators->setConfig( Actuators::ACTUATOR_STEER_REAR, config );
}

S32 BackSteerServo::getServoZero() const {
    return mActuators->getZero( Actuators::ACTUATOR_STEER_REAR );
}

void BackSteerServo::getHallVoltages( Voltage *sensor, double &outVoltage1, double &outVoltage2 ) {
    outVoltage1 = sensor->getChannel( 2 );
    outVoltage2 = sensor->getChannel( 3 );
}
