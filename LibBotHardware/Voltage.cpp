/*
 * Voltage.cpp
 *
 *  Created on: Oct 19, 2009
 *      Author: marks
 */

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

// C/C++
#include <iostream>

// Workspace
#include <Strings.h>

// Project
#include "Voltage.h"
#include "MsgPrint.h"

///////////////////////////////////////////////////////////////////////////////
// IMPLEMENTATION
///////////////////////////////////////////////////////////////////////////////

using namespace std;

//// class Voltage ////////////////////////////////////////////////////////////

Voltage::Voltage( VoltageId id ) : mId( id ) {
}

//// class ads7828voltage /////////////////////////////////////////////////////

// TODO error handling

Ads7828voltage::Ads7828voltage( VoltageId id, U8 chipId, RamaxxConnection * conn ) : Voltage( id ) {
    mName = "Ads7828 voltage " + Strings::ToString((int)chipId );
    mTimeout = 250; // 250 ms message timeout
    mChipId = chipId;
    mChannel.resize( 4 ); // Four channels
    gettimeofday( &mLastUpdate, 0 );

    conn->addQuMsgHandler( MT_ADSVT, *this ); // Receive messages
}

void Ads7828voltage::processQuMessage( const QuMessage &msg ) {
	// Wrong message type or invalid length
    if ( msg.length < 10 ) {
        ERRORPRINT1( "Voltage sensor recived message with invalid length (length %d)", msg.length );
        setSensorState( ERROR );
        return;
    }

    // Check id
    if ( msg.data[0] != mChipId ) {
        return; // Wrong i2c address
    }

    // Check status
    U8 status = msg.data[1];
    if ( status == 0 ) {
        // Valid data, read timestamp and four channels
        /*U32 time = 0;
        time |= ( msg.data[2] << 24 );
        time |= ( msg.data[3] << 16 );
        time |= ( msg.data[4] << 8 );
        time |= msg.data[5];*/

        for ( size_t i = 0; i < mChannel.size(); ++i ) {
            U32 channel = ( msg.data[6 + i * 2] << 8 );
            channel |= msg.data[7 + i * 2];
            mChannel[i] = 2.5 * ((double)channel) / 4096.0;
        }
        mValidMsgCount++;
        setSensorState( VALID );
        gettimeofday( &mLastUpdate, 0 );
        fireDataChanged();
    } else if ( getSensorState() != ERROR ) { // Only print a message if necessary
        cerr << "Voltage sensor reportet an error. Chip id is: " << (int)mChipId << endl;
        setSensorState( ERROR ); // Invalid data
    }
}

//// class Avr32Voltage ////////////////////////////////////////////////////////

Avr32Voltage::Avr32Voltage( RamaxxConnection * conn ) : Voltage( Voltage::AVR ), mValid( false ) {
    mName = "Avr32 voltage";
    mTimeout = 5000; // 1 s message timeout
    mChannel.resize( 8 ); // 8 Channels
    conn->addQuMsgHandler( MT_VOLTAGE, *this );
}

void Avr32Voltage::processQuMessage( const QuMessage &msg ) {
    if ( msg.length < 5 || msg.data[0] > 7 ) {
        ERRORPRINT1( "Avr32 voltage sensor received invalid message (length %d)", msg.length );
        setSensorState( ERROR );
        return;
    }

    U32 milliVolts = ( msg.data[1] << 24 );
    milliVolts |= ( msg.data[2] << 16 );
    milliVolts |= ( msg.data[3] << 8 );
    milliVolts |= msg.data[4];

    mChannel[msg.data[0]] = milliVolts / 1000.0f;
    mValidMsgCount++;
    setSensorState( VALID );
    fireDataChanged();
}
