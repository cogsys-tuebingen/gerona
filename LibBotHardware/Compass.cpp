/*
 * I2cCompass.cpp
 *
 *  Created on: Sep 29, 2009
 *      Author: marks
 */

///////////////////////////////////////////////////////////////////////////////
// I N C L U D E S
///////////////////////////////////////////////////////////////////////////////

// C/C++
#include <iostream>
#include <math.h>

// Workspace
#include <Misc.h>

// Project
#include "Compass.h"
#include "MsgPrint.h"

///////////////////////////////////////////////////////////////////////////////
// D E F I N I T O N S
///////////////////////////////////////////////////////////////////////////////

// Use 15 values to calibrate zero heading
#define CALIB_DATA_MAX 15

///////////////////////////////////////////////////////////////////////////////
// I M P L E M E N T A T I O N
///////////////////////////////////////////////////////////////////////////////

//// class I2cCompass /////////////////////////////////////////////////////////

I2cCompass::I2cCompass( RamaxxConnection * conn ) {
    mName = "Compass i2c";
    mTimeout = 100; // 100 msec message timeout
    mZero = 0;
    mHeading = 0;
    mCalibCount = 0;
    mCalibrated = false;
    conn->addQuMsgHandler( MT_COMPASS, *this );
}

void I2cCompass::processQuMessage( const QuMessage &msg ) {
    // Check message length
    if ( msg.length < 4 ) {
        ERRORPRINT1( "Received compass message with invalid length: %d", (int)msg.length );
        setSensorState( ERROR );
        return;
    }

    // Parse message & check compass state
    U8 state = msg.data[0];
    if (( state & TTypeSpecificCMP::ERROR_STATE ) != 0 ) {
        if ( getSensorState() != ERROR ) {
            ERRORPRINT1( "Compass reported an error. State was: %d", state );
            setSensorState( ERROR );
        }
        return;
    }

    U16 rawHeading = ((msg.data[2] << 8)| msg.data[3]);
    if ( rawHeading < 0 || rawHeading >= 3600 ) {
        // Invalid data
        setSensorState( ERROR );
        ERRORPRINT1( "I2cCompass received invalid data. Heading direction was: %d", rawHeading );
        return;
    }
    double heading = (double)rawHeading / 10.0;
    heading *= M_PI / 180.0;
    mValidMsgCount++;

    // Calibrating?
    if ( mCalibrated ) {
        // Calibration done. Set new data
        setSensorState( VALID );
        mHeading = Misc::normalizeAngle(heading - mZero);
        fireDataChanged();
        updateLogValues();
        return;
    }

    // Still calibrating
    if ( mCalibCount < CALIB_DATA_MAX ) {
        mZero += heading;
        mCalibCount++;
        // Still calibration, dont use data, sensor state will be NOT_INITIALIZED
        return;
    }

    // Finish calibration ( mCalibCount equals CALIB_DATA_MAX )
    mZero /= (double)CALIB_DATA_MAX;
    mCalibrated = true;
    //INFOPRINT( "I2c compass calibration done." );
}

void I2cCompass::writeLogData( LogCollector * logCollector ) {
    logCollector->addValue( COMPASS_LOGID_HEADING, mHeading );
}

void I2cCompass::addLogColumns( LogCollector * logCollector, bool trigger ) {
    logCollector->addColumn( COMPASS_LOGID_HEADING, "Compass heading [rad]", trigger );
}

//// class PniCompass /////////////////////////////////////////////////////////

PniCompass::PniCompass() {
    mName = "PNI compass";
    mTimeout = 500; // 700 msec timeout
    mHeading = mZeroHeading = 0;
    mCalibrated = false;
    mCalibCounter = 0;
}

double PniCompass::getHeading() const {
    return mHeading;
}

void PniCompass::setNewData( double heading ) {
    mValidMsgCount++;
    while ( heading < 0 )
        heading += 2.0*M_PI;
    while ( heading >= 2.0*M_PI )
        heading -= 2.0*M_PI;

    // Calibrating?
    if ( mCalibrated ) {
        mHeading = Misc::normalizeAngle(heading - mZeroHeading);
        setSensorState( VALID );
        fireDataChanged();
        //DEBUGOUT << "PNI compass: " << mHeading << " " << mZeroHeading << endl;
        return;
    }

    // Still calibrating. Dont publish data
    if ( mCalibCounter < CALIB_DATA_MAX ) {
        mZeroHeading += heading;
        mCalibCounter++;
        //DEBUGOUT << mCalibCounter << " " << mCalibrated << endl;
        return;
    }

    // Finish calibration ( mCalibCounter equals CALIB_DATA_MAX )
    mZeroHeading /= (double)CALIB_DATA_MAX;
    mCalibrated = true;
    //INFOPRINT( "PNI compass calibration done." );
}

void PniCompass::setZeroHeading( double zeroHeading ) {
    mZeroHeading = zeroHeading;
}
