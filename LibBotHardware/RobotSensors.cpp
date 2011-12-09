/*
 * Sensors.cpp
 *
 *  Created on: Feb 20, 2010
 *      Author: marks
 */

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

// Project
#include "RobotSensors.h"
#include "QuMessage.h"

///////////////////////////////////////////////////////////////////////////////
// IMPLEMENTATION
///////////////////////////////////////////////////////////////////////////////

//// class Sensors ////////////////////////////////////////////////////////////

RobotSensors::RobotSensors( RamaxxConnection * conn )
    : mRearWheelenc( NULL ), mI2cCompass( NULL ),
      mPtzVoltage( NULL ),
      mSteerVoltage( NULL ),
      mAvrVoltage( NULL ),
      mSonarE0( NULL ), mSonarE2( NULL ), mSonarE4( NULL ),
      mSonarE6( NULL ), mSonarE8( NULL ), mSonarEA( NULL ) // TODO Initialization necessary?
{
    mConn = conn;

    //// Create sensor objects ////

    // Rear wheelencoder
    mRearWheelenc = new WheelEncoder( conn );

    // Built in i2c compass
    mI2cCompass = new I2cCompass( conn );

    // PNI spacepoint compass
    mPniCompass = new PniCompass();

    // Ptz voltage
    mPtzVoltage = new Ads7828voltage( Voltage::PAN_TILT, 0x48, conn );

    // Steer angle voltage
    mSteerVoltage = new Ads7828voltage( Voltage::STEER, 0x4B, conn );

    // Avr voltage sensor
    mAvrVoltage = new Avr32Voltage( conn );

    // Sonar ranger
    mSonarE0 = new SonarRanger( 0xE0, conn );
    mSonarE2 = new SonarRanger( 0xE2, conn );
    mSonarE4 = new SonarRanger( 0xE4, conn );
    mSonarE6 = new SonarRanger( 0xE6, conn );
    mSonarE8 = new SonarRanger( 0xE8, conn );
    mSonarEA = new SonarRanger( 0xEA, conn );

    // Set default sonar ranger position
    RangerPosition rangerPos;
    // Address 0xE0
    rangerPos.x = -0.2;
    rangerPos.y = 0.0;
    rangerPos.a = 0.0;
    mSonarE0->setPosition( rangerPos );
    // Address 0xE2
    rangerPos.y = 0.01;
    mSonarE2->setPosition( rangerPos );
    // Address 0xE4
    rangerPos.y = 0.02;
    mSonarE4->setPosition( rangerPos );
    // Address 0xE6
    rangerPos.y = 0.03;
    mSonarE6->setPosition( rangerPos );
    // Address 0xE8
    rangerPos.y = 0.04;
    mSonarE8->setPosition( rangerPos );
    // Address 0xEA
    rangerPos.y = 0.05;
    mSonarEA->setPosition( rangerPos );

    // Fill vectors
    mCompassVec.push_back( mI2cCompass );
    mCompassVec.push_back( mPniCompass );
    mWheelencVec.push_back( mRearWheelenc );
    mVoltageVec.push_back( mPtzVoltage );
    mVoltageVec.push_back( mSteerVoltage );
    mVoltageVec.push_back( mAvrVoltage );
    mSonarVec.push_back( mSonarE0 );
    mSonarVec.push_back( mSonarE2 );
    mSonarVec.push_back( mSonarE4 );
    mSonarVec.push_back( mSonarE6 );
    mSonarVec.push_back( mSonarE8 );
    mSonarVec.push_back( mSonarEA );
}

RobotSensors::~RobotSensors() {
    delete mRearWheelenc;
    delete mI2cCompass;
    delete mPniCompass;
    delete mSteerVoltage;
    delete mPtzVoltage;
    delete mAvrVoltage;
    delete mSonarE0;
    delete mSonarE2;
    delete mSonarE4;
    delete mSonarE6;
    delete mSonarE8;
    delete mSonarEA;
}

void RobotSensors::runDiagnosis( int timeDiff ) {
    // TODO One list for all sensors?
    for ( size_t i = 0; i < mWheelencVec.size(); ++i )
        mWheelencVec[i]->diagnosis( timeDiff );
    for ( size_t i = 0; i < mVoltageVec.size(); ++i )
        mVoltageVec[i]->diagnosis( timeDiff );
    for ( size_t i = 0; i < mSonarVec.size(); ++i )
        mSonarVec[i]->diagnosis( timeDiff );
    for ( size_t i = 0; i < mCompassVec.size(); ++i )
        mCompassVec[i]->diagnosis( timeDiff );
}

void RobotSensors::setSonarRangerPosition( const RangerPosition &pos, int i ) {
    if ( i < mSonarVec.size() && i > 0 )
        mSonarVec[i]->setPosition( pos );
}

void RobotSensors::setSonarRangerPosition( const RangerPositionVector &pos ) {
    for ( int i = 0; i < pos.size(); ++i ) {
        setSonarRangerPosition( pos[i], i );
    }
}
