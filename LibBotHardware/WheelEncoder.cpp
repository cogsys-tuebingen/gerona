/*
 * WheelEncoder.cpp
 *
 *  Created on: Jan 19, 2010
 *      Author: marks
 */

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

// C/C++
#include <iostream>
#include <cmath>

// Workspace
#include <Global.h>
#include "Stopwatch.h"

// Project
#include "WheelEncoder.h"
#include "Ramaxx.h"
#include "MsgPrint.h"

///////////////////////////////////////////////////////////////////////////////
// IMPLEMENTATION
///////////////////////////////////////////////////////////////////////////////

using namespace std;

//// class WheelEncoder ///////////////////////////////////////////////////////

WheelEncoder::WheelEncoder( RamaxxConnection * conn ) {
    // Member init
    mName = "Wheelencoder";
    for ( int i = 0; i < WHEELENC_SENSOR_COUNT; ++i )
        mDist[i] = mDistSum[i] = mTickSum[i] = mDiagDist[i] = 0;
    mTimeDiff = 0;
    mEnabled[WHEELENC_LEFT] = true;
    mEnabled[WHEELENC_RIGHT] = false;
    mEnableChecked = false;

    // Set default calibration
    WheelEncoderCalib defaultCalib;
    defaultCalib.scaleFwd = defaultCalib.scaleBwd = 0.0032f;
    defaultCalib.enable = false;
    for ( int i = 0; i < WHEELENC_SENSOR_COUNT; ++i )
        mCalib[i] = defaultCalib;
    mCalib[WHEELENC_LEFT].enable = true; // Enable rear left encoder

    // Register to receive messages
    conn->addQuMsgHandler( MT_WHEELENC, *this );
}

bool WheelEncoder::isEnabled( int i ) const {
    if ( i >= WHEELENC_SENSOR_COUNT || i < 0 )
        return false;
    return mEnabled[i];
}

void WheelEncoder::processQuMessage( const QuMessage &msg ) {
    S32 ticks;
    U32 timeDiff = 0;
    double dist;
    bool error = true;

    // Parse message data
    if ( msg.length < 2 * WHEELENC_SENSOR_COUNT + 5 ) {
        // Invalid message length
        setSensorState( ERROR );
        ERRORPRINT( "Received wheel encoder message with invalid length.");
        return;
    }

    // Parse time difference
    timeDiff |= ( msg.data[0] << 24 );
    timeDiff |= ( msg.data[1] << 16 );
    timeDiff |= ( msg.data[2] << 8 );
    timeDiff |= msg.data[3];
    mTimeDiff = 0.001 * (double)timeDiff; // Convert msec to sec

    // Check encoder enabled byte
    U8 enabledByte = msg.data[4];
    mEnabled[WHEELENC_LEFT] = (enabledByte & 0x01) && mCalib[WHEELENC_LEFT].enable;
    mEnabled[WHEELENC_RIGHT] = (enabledByte & 0x02) && mCalib[WHEELENC_RIGHT].enable;

    // Read ticks since last update for each actual sensor
    for ( int i = 0; i < WHEELENC_SENSOR_COUNT; ++i ) {
        if ( !isEnabled( i ))
            continue; // Skip disabled sensors

        // Parse ticks
        ticks = 0;
        ticks |= ( msg.data[i * 4 + 5] << 24 );
        ticks |= ( msg.data[i * 4 + 6] << 16 );
        ticks |= ( msg.data[i * 4 + 7] << 8 );
        ticks |= msg.data[ i * 4 + 8];

        // Set the data
        if ( ticks != 0 ) {
            mTickSum[i] += ticks;
            dist = (double)ticks;
            dist *= ( ticks * mCalib[i].scaleFwd < 0 ) ? mCalib[i].scaleBwd : mCalib[i].scaleFwd;
            mDist[i] = dist;
            mDistSum[i] += dist;
        } else {
            mDist[i] = 0;
        }
    }

    // Need to check enabled flags?
    if ( !mEnableChecked ) {
        // It is possible that the configuration requests to enable an encoder
        // that is disabled on microcontroller level
        if ( mCalib[WHEELENC_LEFT].enable && !(enabledByte & 0x01))
            ERRORPRINT( "Cannot enable the rear left wheel encoder since it is disabled on Avr32 firmware level" );
        if ( mCalib[WHEELENC_RIGHT].enable && !(enabledByte & 0x02))
            ERRORPRINT( "Cannot enable the rear right wheel encoder since it is disabled on Avr32 firmware level" );
        // We assume that the Avr32 firmware does not change during the
        mEnableChecked = true;
    }

    // Validate sensor, fire updates
    if ( mEnabled[WHEELENC_LEFT] || mEnabled[WHEELENC_RIGHT] ) {
        setSensorState( VALID );
        mValidMsgCount++;
        fireDataChanged();
        error = false;
    } else if ( getSensorState() != ERROR ){
        // No wheel encoder enabled
        setSensorState( ERROR );
        ERRORPRINT( "No wheel encoder enabled." );
    }
}

void WheelEncoder::diagnosis( int timeDiff ) {
    Sensor::diagnosis( timeDiff );

    if ( getSensorState() == Sensor::VALID && isLeftEnabled() && isRightEnabled()) {
        double leftDist = mDistSum[WHEELENC_LEFT] - mDiagDist[WHEELENC_LEFT];
        double rightDist = mDistSum[WHEELENC_RIGHT] - mDiagDist[WHEELENC_RIGHT];

        if ( fabs( leftDist ) < 10.0 && fabs( rightDist ) < 10.0 )
            return;

        if ( fabs( leftDist - rightDist ) > 5.0 )
            ERRORPRINT1( "Wheelencoder sensors reporting strange looking data (difference %f meter).",
                         fabs( leftDist - rightDist ));

        mDiagDist[WHEELENC_LEFT] = mDistSum[WHEELENC_LEFT];
        mDiagDist[WHEELENC_RIGHT] = mDistSum[WHEELENC_RIGHT];
    }
}

void WheelEncoder::setCalib( const WheelEncoderCalib &calib, int i ) {
    mCalib[i] = calib;
    mEnableChecked = false; // Recheck enable flags compatibility
}
