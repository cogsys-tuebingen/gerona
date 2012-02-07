
///////////////////////////////////////////////////////////////////////////////
// I N C L U D E S
///////////////////////////////////////////////////////////////////////////////

// Project
#include "Avr32Params.h"

///////////////////////////////////////////////////////////////////////////////
// D E F I N I T I O N S
///////////////////////////////////////////////////////////////////////////////

// Parameter types
#define MSG_PARAM_WHEELENC_CALIB                001
#define MSG_PARAM_ACTUATOR                      200
#define MSG_PARAM_SAVE                          255

// Parameter subtypes
#define MSG_PARAM_WHEELENC_REAR_LEFT            001
#define MSG_PARAM_WHEELENC_REAR_RIGHT           002
#define MSG_PARAM_ACTU_MIN                      001
#define MSG_PARAM_ACTU_MAX                      002
#define MSG_PARAM_ACTU_DEFAULT                  003
#define MSG_PARAM_ACTU_SPEED                    004

///////////////////////////////////////////////////////////////////////////////
// I M P L E M E N T A T I O N
///////////////////////////////////////////////////////////////////////////////


Avr32Params::Avr32Params( RamaxxConnection *conn, RobotSensors *sensors, Actuators *actuators )
    : mConn( conn ), mSensors( sensors ), mActuators( actuators ), mParamsSend( false ) {

    // Register to receive event notifications
    mConn->addQuMsgHandler( MT_PARAM, *this );

    // Check if the connections is established
    if ( mConn->isOpen()) {
        sendAllParams();
    }
}

void Avr32Params::connectionEstablished() {
    // Send params if necessary
    if ( !mParamsSend ) {
        sendAllParams();
    }
}

void Avr32Params::sendAllParams() {
    // Send wheelencoder calibration
    WheelEncoderCalib wheelEncCalib =
        mSensors->getRearWheelEncoder()->getLeftCalib();
    sendParam( MSG_PARAM_WHEELENC_CALIB, MSG_PARAM_WHEELENC_REAR_LEFT,
               (S32)(wheelEncCalib.scaleFwd * 10E5));
    wheelEncCalib =
        mSensors->getRearWheelEncoder()->getRightCalib();
    sendParam( MSG_PARAM_WHEELENC_CALIB, MSG_PARAM_WHEELENC_REAR_RIGHT,
               (S32)(wheelEncCalib.scaleFwd * 10E5));

    // Actuator min, max, default and speed
    ActuatorConfig actuConf;
    for ( int i = 0;  i < Actuators::ACTUATOR_COUNT; ++i ) {
        actuConf = mActuators->getConfig((Actuators::ActuatorId)i);
        sendParam( MSG_PARAM_ACTUATOR + i, MSG_PARAM_ACTU_MIN, actuConf.min );
        sendParam( MSG_PARAM_ACTUATOR + i, MSG_PARAM_ACTU_MAX, actuConf.max );
        sendParam( MSG_PARAM_ACTUATOR + i, MSG_PARAM_ACTU_DEFAULT, actuConf.zero );
        sendParam( MSG_PARAM_ACTUATOR + i, MSG_PARAM_ACTU_SPEED, actuConf.speed * 1000 );
    }

    // Send write to flash request
    sendParam( MSG_PARAM_SAVE, 0, 0 );

    mParamsSend = true;
}

void Avr32Params::sendParam( const U8& type, const U8& subtype, const S32& val ) {
    // Create message
    QuMessage msg;
    msg.type = MT_PARAM;
    msg.length = 6;
    msg.data.resize( 6 );

    // Set parameter type and subtype
    msg.data[0] = type;
    msg.data[1] = subtype;

    // Set value
    msg.data[2] = (val >> 24) & 0xFF;
    msg.data[3] = (val >> 16) & 0xFF;
    msg.data[4] = (val >> 8) & 0xFF;
    msg.data[5] = val & 0xFF;

    // Enqueue message
    mConn->queueMsg( msg );
}
