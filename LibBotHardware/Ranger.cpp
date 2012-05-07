
///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

// C/C++
#include <iostream>
#include <sstream>

// Workspace
#include <Strings.h>

// Project
#include "Ranger.h"

///////////////////////////////////////////////////////////////////////////////
// CONSTANTS
///////////////////////////////////////////////////////////////////////////////

// Log ids
const std::string SONAR_LOGID_RANGE = "SonarRanger.range";
const std::string IR_LOGID_RANGE = "IrRanger.range";

///////////////////////////////////////////////////////////////////////////////
// IMPLEMENTATION
///////////////////////////////////////////////////////////////////////////////

using namespace std;

//// class Ranger /////////////////////////////////////////////////////////////

Ranger::Ranger( RangerType type, RangerPosition pos )
        : mPos( pos ), mRange( 0.0f ), mType( type ) {
}

void Ranger::setRange( float range, bool valid ) {
    mRange = range;
}

void Ranger::getRange( float &x, float &y, float &z ) const {
    x = mPos.x; y = mPos.y; z = mPos.z;
    x += mRange*cos( mPos.a );
    y += mRange*sin( mPos.a );
}

//// class SonarRanger ////////////////////////////////////////////////////////

// TODO Error handling (values are always valid at current state)

SonarRanger::SonarRanger( U8 i2cAddress, RamaxxConnection * conn, RangerPosition pos )
        : Ranger( Ranger::SONAR, pos ), mAddress( i2cAddress ) {
    mName = "Us ranger "+ Strings::ToString((int)i2cAddress );
    mTimeout = 500; // 500 msec message timeout

    // Register to receive messages
    conn->addQuMsgHandler( MT_SONAR, *this );
}

void SonarRanger::processQuMessage( const QuMessage &msg ) {
    if ( msg.data[0] != mAddress )  {
        return; // I2c address does not match
    }
    // Parse range
    // TODO Range check?
    unsigned int dist = 0;
    ((unsigned char *) &(dist))[0] = msg.data[2];
    ((unsigned char *) &(dist))[1] = msg.data[1];  
    setRange((float)dist / 100.0f, true );
    setSensorState( VALID );
    mValidMsgCount++;
    fireDataChanged();
    updateLogValues();
}

void SonarRanger::setPosition( const RangerPosition &pos ) {
    mPos = pos;
}

void SonarRanger::addLogColumns( LogCollector * logCollector, bool trigger ) {
    stringstream desc;
    desc << "Sonar ranger with address " << (int)mAddress << " range [m]";
    logCollector->addColumn( SONAR_LOGID_RANGE, desc.str(), trigger );
}

void SonarRanger::writeLogData( LogCollector * logCollector ) {
    logCollector->addValue( SONAR_LOGID_RANGE, getRange());
}

//// class IrRanger ///////////////////////////////////////////////////////////

IrRanger::IrRanger( U8 rangerId, RangerPosition pos, RamaxxConnection * conn )
                    : Ranger( Ranger::IR, pos ), mId( rangerId ) {
    mName = "Ir ranger " + Strings::ToString((int)rangerId );
    mTimeout = 400; // 500 msec message timeout

    // Register to receive messages
    conn->addQuMsgHandler( MT_IR, *this );
}

void IrRanger::processQuMessage( const QuMessage &msg ) {
    if ( ((int)msg.data[0]) - 2 != mId ) {
        return; // Not the id of this sensor. Skip message
    }

    // TODO Range check?
    unsigned int dist = 0;
    ((unsigned char *) &(dist))[0] = msg.data[2];
    ((unsigned char *) &(dist))[1] = msg.data[1];
    setRange((float)dist / 100.0f, true );
    setSensorState( VALID );
    mValidMsgCount++;
    fireDataChanged();
    updateLogValues();
}

void IrRanger::addLogColumns( LogCollector * logCollector, bool trigger ) {
    stringstream desc;
    desc << "Ir ranger number " << (int)mId << " range [m]";
    logCollector->addColumn( IR_LOGID_RANGE, desc.str(), trigger );
}

void IrRanger::writeLogData( LogCollector * logCollector ) {
    logCollector->addValue( IR_LOGID_RANGE, getRange());
}
