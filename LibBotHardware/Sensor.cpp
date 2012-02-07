
////////////////////////////////////////////////////////////////////////////////
// INCLUDES
////////////////////////////////////////////////////////////////////////////////

// Project
#include "Sensor.h"
#include "MsgPrint.h"

////////////////////////////////////////////////////////////////////////////////
// IMPLEMENTATION
////////////////////////////////////////////////////////////////////////////////

//// class Sensor //////////////////////////////////////////////////////////////

Sensor::Sensor() {
    mState = NOT_INITIALIZED;
    mValidMsgCount = 0;
    mDiagnosisTimeDiff = 0;
    mTimeout = 150; // 150 msec default diagnosis time
    setValidData( false );
}

Sensor::~Sensor() {
    // Nothing to do
}

void Sensor::setSensorState( const SensorState &state ) {
    if ( mState != state ) {
        if ( state == ERROR || state == TIMEOUT ) {
            ERRORPRINT3( "Sensor \"%s\" switched state from \"%s\" to \"%s\"",
                         mName.c_str(),
                         stateToString( mState ).c_str(),
                         stateToString( state ).c_str());
            mState = state;
            if ( !hasValidData()) fireStateChanged(); // Force a state changed event here
            else setValidData( false );
        } else if ( state == VALID ) {
            /*INFOPRINT3( "Sensor \"%s\" switched state from \"%s\" to \"%s\"",
                        mName.c_str(),
                        stateToString( mState ).c_str(),
                        stateToString( state ).c_str());*/
            mState = state;
            if ( hasValidData()) fireStateChanged(); // Force a state changed event here
            else setValidData( true );
        } else { // State is "not initialized"
            mState = state;
            if ( !hasValidData()) fireStateChanged(); // Force a state changed event here
            else setValidData( false );
        }
    }
}

void Sensor::diagnosis( int timeDiff ) {
    if ( mDiagnosisTimeDiff + timeDiff > mTimeout ) {
        // Valid message received?
        if ( mValidMsgCount <= 0 && mState != ERROR ) {
            setSensorState( TIMEOUT );
        }
        mDiagnosisTimeDiff = 0;
        mValidMsgCount = 0;
    } else {
        mDiagnosisTimeDiff += timeDiff;
    }
}

std::string Sensor::stateToString( const SensorState &state ) {
    switch ( state ) {
    case NOT_INITIALIZED:
        return "not initialized";
    case ERROR:
        return "error";
    case TIMEOUT:
        return "timeout";
    case VALID:
        return "valid";
    }
}
