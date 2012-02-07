/*
 * On driver and client side used data types and definitions.
 *
 * 15.10.2010 Karsten Bohlmann, Henrik Marks
 */

#ifndef ROBOT_H
#define ROBOT_H

////////////////////////////////////////////////////////////////////////////////
// INCLUDES
////////////////////////////////////////////////////////////////////////////////

// Workspace
#include <Global.h>

////////////////////////////////////////////////////////////////////////////////
// DEFINITIONS/DECLARATIONS
////////////////////////////////////////////////////////////////////////////////

namespace Ra {

enum RamaxxCmd {
    CMD_STEER_FRONT_RAD = 1,
    CMD_STEER_FRONT_SERVO = 2,
    CMD_STEER_REAR_RAD = 3,
    CMD_STEER_REAR_SERVO = 4,
    CMD_SPEED_MSEC = 5,
    CMD_SPEED_SERVO = 6
};

/** The available steer modes. */
enum RobotSteerMode { FRONT = 1,
                      REAR = 2,
                      BOTH = 3,
                      PARALLEL = 4
};

/** Command modes */
enum RobotCmdMode {
    MANUAL = 1,             // Joystick only
    MANUAL_EXCEPT_PTZ = 2,  // Joystick only but allow automatic ptz commands
    AUTO = 3                // Allow automatic commands
};

/**
 * This data structure contains
 * status information of a robots
 * current configuration.
 *
 * It is used for low level data transmission.
 */
struct InternalStateData {

    /** Misc status flags bitmasks */
    static const int HAS_ENCODER_LEFT = 1;              // Rear left encoder is active and has valid data
    static const int HAS_ENCODER_RIGHT = ( 1 << 1 );    // Rear right encoder is active and has valid data
    static const int HAS_STEER_FRONT = ( 1 << 2 );      // Currently always true
    static const int HAS_STEER_BACK = ( 1 << 3 );       // Rear axis steerable?
    static const int IS_MANUAL_MODE = ( 1 << 4 );       // Manual mode enabled (joystick active)
    static const int IS_AVR32_CONNECTED = ( 1 << 5 );   // True if there is a connection to the microcontroller

    /** Command id */
    static const int CMD_STEERMODE = 1;

    /** Current steer mode. */
    RobotSteerMode steerMode;

    /** Misc flags */
    Uint  robotFlags;
};

/**
 * Defines accu voltage channel numbers.
 */
struct AccuVoltageIndex {
    /** Channel indices */
    static const int PC = 0;
    static const int MOTOR = 1;

    /** Number of accu voltage channels */
    static const int ACCU_VOLTAGE_COUNT = 2;
};

/**
 * Defines servo voltage channel numbers.
 */
struct ServoVoltageIndex {
    /** Channel numbers */
    static const int STEER_FRONT1 = 0;
    static const int STEER_FRONT2 = 1;
    static const int STEER_REAR1 = 2;
    static const int STEER_REAR2 = 3;
    static const int PAN = 4;
    static const int TILT = 5;

    /** Number of available servo voltages. */
    static const int VOLTAGE_COUNT = 6;
};

/**
 * Defines servo actuator indices.
 */
struct ServoIndex {
    /** Actuator index */
    static const int SPEED = 0;
    static const int STEER_FRONT = 1;
    static const int STEER_REAR = 2;
    static const int PAN = 3;
    static const int TILT = 4;

    /** Number of available servos. */
    static const int SERVO_COUNT = 5;
};

/**
 * Holds raw odometry data and defines field indices.
 */
struct RawOdometryData {
    /** Indices definitions */
    static const int ENCODER_LEFT = 0;
    static const int ENCODER_RIGHT = 1;
    static const int STEER_FRONT = 2;
    static const int STEER_REAR = 3;
    static const int COMPASS = 4;
    static const int TRIGGER = 5;
    static const int ENCODER_LEFT_TICKS = 6;
    static const int ENCODER_RIGHT_TICKS = 7;

    static const int ENCODER_TRIGGER = 1.0f;
    static const int COMPASS_TRIGGER = 0.0f;
    static const int BOTH_TRIGGER = -1.0f;

    /** Ticks sum left encoder [Ticks] */
    float encoderLeftTicks;
    /** Ticks sum right encoder [Ticks] */
    float encoderRightTicks;
    /** Left encoder distance [m] */
    float encoderLeft;
    /** Right encoder distance [m] */
    float encoderRight;

    /** Front steer angle [rad] */
    float steerFrontRad;
    /** Back steer angle [rad] */
    float steerRearRad;

    /** Compass heading direction [rad] */
    float compassRad;

    /** Did the encoder or the compass trigger the update? */
    int trigger;

    /**
     * Sets the data accordantly to the given player data.
     *
     * @param voltages The received volate data (8 elements).
     */
    void SetData( std::vector<float>& voltages ) {
        if ( voltages.size() >= GetDataSize()) {
            encoderLeft = voltages[ENCODER_LEFT];
            encoderLeftTicks = voltages[ENCODER_LEFT_TICKS];
            encoderRight = voltages[ENCODER_RIGHT];
            encoderRightTicks = voltages[ENCODER_RIGHT_TICKS];
            steerFrontRad = voltages[STEER_FRONT];
            steerRearRad = voltages[STEER_REAR];
            compassRad = voltages[COMPASS];
            trigger = (int)voltages[TRIGGER];

        } else {
            // TODO error handling ?
        }
    }


    static Uint GetDataSize() {
        return 8;
    }
};

}; // Namespace Ra

#endif // ROBOT_H
