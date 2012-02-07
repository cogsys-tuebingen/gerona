/*
 * AdnsOdometry.cpp
 *
 *  Created on: Sep 15, 2009
 *      Author: marks
 */

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

// C/C++
#include <iostream>
#include <math.h>
#include <sstream>

// GSL
#include <gsl/gsl_blas.h>

// Project
#include "Adns.h"
#include "Ramaxx.h"

///////////////////////////////////////////////////////////////////////////////
// IMPLEMENTATION
///////////////////////////////////////////////////////////////////////////////

//// class AdnsChip ///////////////////////////////////////////////////////////

AdnsChip::AdnsChip() {
    reset();
}

void AdnsChip::reset() {
    mSqual = mDx = mDy = mDxSum = mDySum = 0;
}

void AdnsChip::set( int dx, int dy, int squal ) {
    mDx = dx;
    mDy = dy;
    mSqual = squal;
    mDxSum += mDx;
    mDySum += mDy;
}

//// class AdnsSensor /////////////////////////////////////////////////////////

AdnsSensor::AdnsSensor( U8 sensorId, size_t chipCount, RamaxxConnection * conn ) {
    mSensorId = sensorId;
    mChips.resize( chipCount );

    U8 msgTypes[] = { MT_ADNS_POSITION, MT_ADNS_ERROR, MT_ADNS_PIXDUMP };
    conn->addQuMsgHandler( msgTypes, 3, *this );
}

void AdnsSensor::processQuMessage( const QuMessage &msg ) {
    switch ( msg.type ) {
    case MT_ADNS_POSITION:
        processPositionMsg( msg );
        break;
    case MT_ADNS_PIXDUMP:
        //processDumpMsg( msg );
        break;
    case MT_ADNS_ERROR:
        //processErrorMsg( msg );
        break;
    default:
        break;
    }
}

bool AdnsSensor::processPositionMsg( const QuMessage &msg ) {
    // Check message length
    if ( msg.length < 5 + 5 * getChipCount()) {
        cout << "Adns: ERROR. Received message with invalid length. Skipping." << endl;
        return false; // Message length should be at least ten bytes
    }

    // Check id
    if ( msg.data[0] != mSensorId ) {
        return false; // Wrong sensor id
    }

    // Parse timestamp
    U32 time = 0;
    time |= ( msg.data[1] << 24 );
    time |= ( msg.data[2] << 16 );
    time |= ( msg.data[3] << 8 );
    time |= msg.data[4];

    // Parse data
    for ( size_t i = 0; i < getChipCount(); ++i ) {
        size_t dataPos = ( 5 * i ) + 5;
        S16 dx = 0;
        dx |= ( msg.data[dataPos] << 8 );
        dx |= ( msg.data[dataPos + 1] );
        S16 dy = 0;
        dy |= ( msg.data[dataPos + 2] << 8 );
        dy |= ( msg.data[dataPos + 3] );
        U8 sq = msg.data[dataPos + 4];
        getChip( i )->set( dx, dy, sq );
        cout << "Adns: " << (int)dx << " " << (int)dy << " " << (int)sq << endl;
    }

    return true;
}



//// class AdnsSensorId ///////////////////////////////////////////////////////

AdnsSensorId::AdnsSensorId( Uint sensorId, Uint chipNumber ) {
    mSensorId = sensorId;
    mChipNumber = chipNumber;
}

bool AdnsSensorId::isLocallyConnected() {
    return ( mSensorId & 0x80 ) == 0;
}

//// class AdnsError //////////////////////////////////////////////////////////

AdnsError::AdnsError( const QuMessage& msg ) {
    mErrorId = msg.data[0];
    mId.setSensorId( msg.data[1] );
    mId.setChipNumber( msg.data[2] );
    mValue = msg.data[3];
}

string AdnsError::toString() const {
    ostringstream os;
    switch ( mErrorId ) {
    case ADNS_ERROR_STATUS:
        os << "Chip number " << mId.getChipNumber();
        os << " at sensor with id " << mId.getSensorId();
        os << " reported status " << mValue;
        break;
    case ADNS_ERROR_PIXDUMP:
        os << "Chip number " << mId.getChipNumber();
        os << " at sensor with id " << mId.getSensorId();
        os << " reported an pixdump error.";
        break;
    case ADNS_ERROR_TWI:
        os <<  "Sensor with id " << mId.getSensorId();
        os << " reported a general TWI error.";
        break;
    case ADNS_ERROR_CHIP_NOT_READY:
        os << "Sensor with id " << mId.getSensorId();
        os << " reported a chip not ready error.";
        break;
    default:
        os << "An unknown error occured. Error id is " << mErrorId;
    }
    return os.str();
}

//// class AdnsPositionData ///////////////////////////////////////////////////

AdnsPositionData::AdnsPositionData()
    : mId( 0, 0 ) {
    mDx = mDy = mSq = 0;
}

bool AdnsPositionData::isValid() const {
    return
        abs( mDx ) < 255 &&
        abs( mDy ) < 255 &&
        mSq < 255;
}

//// Class AdnsCommunication //////////////////////////////////////////////////////

AdnsCommunication::AdnsCommunication( RamaxxConnection * conn ) {
    U8 msgTypes[] = { MT_ADNS_PIXDUMP, MT_ADNS_ERROR, MT_ADNS_POSITION };
    conn->addQuMsgHandler( msgTypes, 3, *this );
}

void AdnsCommunication::processQuMessage( const QuMessage &msg ) {
    switch ( msg.type ) {
    case MT_ADNS_POSITION:
        processPositionMsg( msg );
        break;
    case MT_ADNS_PIXDUMP:
        processDumpMsg( msg );
        break;
    case MT_ADNS_ERROR:
        processErrorMsg( msg );
        break;
    default:
        break;
    }
}

void AdnsCommunication::processErrorMsg( const QuMessage& msg ) {
    // Parse error
    AdnsError error( msg );
    // Notify listeners
    for ( size_t i = 0; i < mListeners.size(); ++i ) {
        mListeners[i]->processError( error );
    }
}

void AdnsCommunication::processPositionMsg( const QuMessage& msg ) {
    size_t numSensors = ( msg.length / 5 ) - 1;

    // Resize position data vector if necessary
    if ( numSensors > mPositionList.size()) {
        mPositionList.resize( numSensors );
    }

    // Parse id and time
    AdnsSensorId id( msg.data[0], 0 );
    Uint time = 0;

    // Parse position data
    parsePositionMsg( msg, mPositionList, time );

    // Notify listeners
    for ( size_t i = 0; i < mListeners.size(); ++i ) {
        mListeners[i]->processPositionData( id, time, mPositionList );
    }
}

bool AdnsCommunication::parsePositionMsg( const QuMessage &msg, vector<AdnsPositionData> &data, U32 &time ) {
    if ( msg.length < 10 ) {
        return false; // Message length should be at least ten bytes
    }

    Uint numSensors = ( msg.length / 5 ) - 1; // Five bytes sensor id and timestamp, five bytes for each sensor

    // Parse id and timestamp
    AdnsSensorId id( msg.data[0], 0 );
    time = 0;
    time |= ( msg.data[1] << 24 );
    time |= ( msg.data[2] << 16 );
    time |= ( msg.data[3] << 8 );
    time |= msg.data[4];

    // Resize data vector if necessary
    if ( data.size() != numSensors ) {
        data.resize( numSensors );
    }

    // Parse data
    for ( Uint i = 0; i < numSensors; ++i ) {
        Uint dataPos = ( 5 * i ) + 5;
        S16 dx = 0;
        dx |= ( msg.data[dataPos] << 8 );
        dx |= ( msg.data[dataPos + 1] );
        S16 dy = 0;
        dy |= ( msg.data[dataPos + 2] << 8 );
        dy |= ( msg.data[dataPos + 3] );
        U8 sq = msg.data[dataPos + 4];
        cout << "SQ: " << (int)sq << endl;
        id.setChipNumber( i );
        data[i].set( id, dx, dy, sq );
    }

    return true;
}

void AdnsCommunication::processDumpMsg( const QuMessage& msg ) {
    AdnsSensorId id;

    if ( msg.length < 2 )
        return;

    // Parse id
    id.setSensorId( msg.data[0] );
    id.setChipNumber( msg.data[1] );

    // Copy data
    // TODO: Remove
    vector<int> pixelData( msg.length - 2 );
    for ( int i = 2; i < msg.length; ++i ) {
        pixelData[i - 2] = msg.data[i];
    }

    // Notify listener
    for ( size_t i = 0; i < mListeners.size(); ++i ) {
        mListeners[i]->processPixdumpData( id, pixelData );
    }
}

void AdnsCommunication::setSensorMode( AdnsSensorId& id, int mode ) {
    U8 buffer[3];

    buffer[0] = (U8)mode;
    buffer[1] = (U8)id.getSensorId();
    buffer[2] = (U8)id.getChipNumber();
    QuMessage msg( MT_ADNS_MODE, 3, buffer );
    Ramaxx::getRamaxx()->queueMsg( msg );
}

void AdnsCommunication::addAdnsListener( AdnsListener * lis ) {
    mListeners.push_back( lis );
}

void AdnsCommunication::removeAdnsListener( AdnsListener * lis ) {
    // TODO Implement
}

//// Static functions ////////////////////////////////////////////////////////


double adnsNormalizeRadian( double arg ) {
    if ( arg >= ( 2.0 * M_PI ))
        arg -= ( 2.0 * M_PI );
    else if ( arg <= ( 2.0 * M_PI ))
        arg += ( 2.0 * M_PI );
    return arg;
}

