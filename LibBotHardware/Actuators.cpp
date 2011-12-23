/*
 * Actuators.cpp
 *
 *  Created on: Feb 22, 2010
 *      Author: marks
 */

///////////////////////////////////////////////////////////////////////////////
// I N C L U D E S
///////////////////////////////////////////////////////////////////////////////

// Project
#include "MsgPrint.h"
#include "Actuators.h"

///////////////////////////////////////////////////////////////////////////////
// D E F I N T I O N S
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// I M P L E M E N T A T I O N
///////////////////////////////////////////////////////////////////////////////

//// class Actuators //////////////////////////////////////////////////////////

Actuators::Actuators( RamaxxConnection * conn ) {
    mConn = conn;

    // Set default configuration and initial values
    ActuatorConfig defaultConfig;
    for ( int i = 0; i < ACTUATOR_COUNT; ++i ) {
        mActuStatus[i].config = defaultConfig;
        mActuStatus[i].presentPosition = -1; // Unknown
        mActuStatus[i].requestedPosition = defaultConfig.zero;
        mActuStatus[i].error = false; // No error reported
        mActuStatus[i].hasPositionFeedback = false; // No position feedback received
    }

    // Set speed servo default config
    defaultConfig.min = 700;
    defaultConfig.zero = 2250;
    defaultConfig.max = 3000;
    mActuStatus[ACTUATOR_SPEED].config = defaultConfig;

    // Registzer to receive position feedback messages
    mConn->addQuMsgHandler( MT_ACTUATOR_PRESENT_POS, *this );
}

void Actuators::processQuMessage( const QuMessage &msg ) {
    // Check message length
    if ( msg.length < 5 ) {
        ERRORPRINT1( "Received actuator feedback message with wrong size. Size was: %d", (int)msg.data.size());
        return;
    }

    // Check actuator id
    if ( msg.data[0] >= ACTUATOR_COUNT ) {
        ERRORPRINT1( "Received actuator feedback message with invalid actuator id. Id was: %d", (int)msg.data[0]);
        return;
    }

    // Parse message and set data
    ActuatorId actId = (ActuatorId)msg.data[0];
    S32 presentPos = msg.data[4];
    presentPos |= msg.data[3] << 8;
    presentPos |= msg.data[2] << 16;
    presentPos |= msg.data[1] << 24;
    mActuStatus[actId].presentPosition = presentPos;
    mActuStatus[actId].hasPositionFeedback = true;
}

void Actuators::setPosition( ActuatorId id, S32 value ) {
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

    // save a copy of the requested value
    mActuStatus[id].requestedPosition = value;
}

void Actuators::setConfig( ActuatorId id, const ActuatorConfig &config ) {
    mActuStatus[id].config = config;
}

void Actuators::setMovingSpeed( ActuatorId id, double speed ) {
    // Create and enqueue message
    QuMessage msg;
    msg.type = MT_ACTUATOR_SPEED;
    msg.length = 5;
    msg.data.resize( msg.length );
    msg.data[0] = id;
    S32 value = speed * 1000.0;
    msg.data[1] = ( value >> 24 ) & 0xFF;
    msg.data[2] = ( value >> 16 ) & 0xFF;
    msg.data[3] = ( value >> 8 ) & 0xFF;
    msg.data[4] = value & 0xFF;
    mConn->queueMsg( msg );

    // Keep a copy of the requested speed value
    mActuStatus[id].requestedSpeed = speed;
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
    setPosition( ACTUATOR_STEER_REAR, value );
}

void Actuators::setSteerFront( S32 value ) {
    setPosition( ACTUATOR_STEER_FRONT, value );
}

void Actuators::setSpeed( S32 value ) {
    setPosition( ACTUATOR_SPEED, value );
}

void Actuators::setCamPan( S32 value ) {
    setPosition( ACTUATOR_CAM_PAN, value );
}

void Actuators::setCamTilt( S32 value ) {
    setPosition( ACTUATOR_CAM_TILT, value );
}

S32 Actuators::getRequestedSteerBack() const {
    return getRequestedPosition( ACTUATOR_STEER_REAR );
}

S32 Actuators::getRequestedSteerFront() const {
    return getRequestedPosition( ACTUATOR_STEER_FRONT );
}

S32 Actuators::getRequestedSpeed() const {
    return getRequestedPosition( ACTUATOR_SPEED );
}

S32 Actuators::getRequestedCamPan() const {
    return getRequestedPosition( ACTUATOR_CAM_PAN );
}

S32 Actuators::getRequestedCamTilt() const {
    return getRequestedPosition( ACTUATOR_CAM_TILT );
}
