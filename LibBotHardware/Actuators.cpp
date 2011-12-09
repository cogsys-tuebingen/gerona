/*
 * Actuators.cpp
 *
 *  Created on: Feb 22, 2010
 *      Author: marks
 */

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

// Project
#include "Actuators.h"
#include "QuMessage.h"

///////////////////////////////////////////////////////////////////////////////
// DEFINTIONS
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// IMPLEMENTATION
///////////////////////////////////////////////////////////////////////////////

//// class Actuators //////////////////////////////////////////////////////////

Actuators::Actuators( RamaxxConnection * conn ) {
    mConn = conn;

    // Set default configuration and initial values
    ActuatorConfig defaultConfig;
    for ( int i = 0; i < ACTUATOR_COUNT; ++i ) {
        mConfig[i] = defaultConfig;
        mCurrentValue[i] = -1; // Unknown
    }

    // Set speed servo default config
    defaultConfig.min = 700;
    defaultConfig.zero = 2250;
    defaultConfig.max = 3000;
    mConfig[ACTUATOR_SPEED] = defaultConfig;
}

void Actuators::setActuator( ActuatorId id, S32 value ) {
    QuMessage msg;
    msg.type = MT_ACTUATOR;
    msg.length = 5;
    msg.data.resize( msg.length );
    msg.data[0] = id;
    msg.data[1] = ( value >> 24 ) & 0xFF;
    msg.data[2] = ( value >> 16 ) & 0xFF;
    msg.data[3] = ( value >> 8 ) & 0xFF;
    msg.data[4] = value & 0xFF;
    mConn->queueMsg( msg );

    // save a copy of the current value
    mCurrentValue[id] = value;
}

void Actuators::setActuatorConfig( ActuatorId id, const ActuatorConfig &config ) {
    mConfig[id] = config;
    // TODO Check range? Set speed!
}

void Actuators::setActuatorSpeed( ActuatorId id, double speed ) {
    mConfig[id].speed = speed;
    // not yet implemented
}

void Actuators::beep( const int *beeps ) const {
    U8 count = 0;
    while (count < 255 && beeps[count] )
        ++count;
    QuMessage msg;
    msg.type = MT_BEEP;
    msg.length = count * 2 + 2;
    msg.data.resize( msg.length );
    msg.data[0] = count;
    for ( int i = 0; i < count; ++i ) {
        U16 val = beeps[i];
        val = SWAP16( val );
        *(U16*)&( msg.data[ i * 2 + 2 ] ) = val;
    }
    mConn->queueMsg( msg );
}

void Actuators::setSteerBack( S32 value ) {
    setActuator( ACTUATOR_STEER_REAR, value );
}

void Actuators::setSteerFront( S32 value ) {
    setActuator( ACTUATOR_STEER_FRONT, value );
}

void Actuators::setSpeed( S32 value ) {
    setActuator( ACTUATOR_SPEED, value );
}

void Actuators::setCamPan( S32 value ) {
    setActuator( ACTUATOR_CAM_PAN, value );
}

void Actuators::setCamTilt( S32 value ) {
    setActuator( ACTUATOR_CAM_TILT, value );
}

S32 Actuators::getSteerBack() const {
    return getActuator( ACTUATOR_STEER_REAR );
}

S32 Actuators::getSteerFront() const {
    return getActuator( ACTUATOR_STEER_FRONT );
}

S32 Actuators::getSpeed() const {
    return getActuator( ACTUATOR_SPEED );
}

S32 Actuators::getCamPan() const {
    return getActuator( ACTUATOR_CAM_PAN );
}

S32 Actuators::getCamTilt() const {
    return getActuator( ACTUATOR_CAM_TILT );
}
