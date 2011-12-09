/*
 * Actuators.h
 *
 *  Created on: Feb 22, 2010
 *      Author: Marks, Masselli
 */

#ifndef ACTUATORS_H_
#define ACTUATORS_H_

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

// C/C++
#include <string>

// Workspace
#include "Global.h"

// Project
#include "UsbConn.h"
#include "LogAdapter.h"

///////////////////////////////////////////////////////////////////////////////
// DEFINITIONS
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// DECLARATIONS
///////////////////////////////////////////////////////////////////////////////

/**
 * Represents the configuration of a servo-like actuator.
 */
struct ActuatorConfig {
    /**
     * Create a default configuration.
     */
    ActuatorConfig() :
            zero( 2250 ), min( 1500 ), max( 3000 ), speed( 60.0f/0.17f )
    {}

    /// Default value
    S32 zero;
    /// Minimum value
    S32 min;
    /// Maximum value
    S32 max;

    /**
     * Turning velocity in radian per sec. NOTE: This value may be used to
     * calculate the current position of the servo
     * (e.g. calculating the current steer angle) but it does not
     * affect the actual turning velocity if the servo is a simple
     * RC-servo or motor controller.
     * It's only possible to change the turning velocity of an
     * Dynamixel AX12 servo.
     */
    double speed;
};

/**
 * Represents the robots actuators.
 */
class Actuators {
public:

    /**
     * Actuator identification numbers. Used for data transmission,
     * dont change!
     */
    enum ActuatorId {
        ACTUATOR_STEER_FRONT = 0,
        ACTUATOR_STEER_REAR = 1,
        ACTUATOR_SPEED = 2,
        ACTUATOR_CAM_PAN = 3,
        ACTUATOR_CAM_TILT = 4,
        ACTUATOR_LASER_ROLL = 5,
        ACTUATOR_LASER_TILT = 6,
        ACTUATOR_EXT = 7,
        ACTUATOR_COUNT = 8 // Dummy entry represents the number of available actuators
    };

    /**
     * Constructor.
     *
     * @param conn The connection to the robot.
     */
    Actuators( RamaxxConnection * conn );

    /**
     * Sets a actuator position.
     *
     * @param Id Id of the actuator.
     * @param value New actuator position.
     */
    void setActuator( ActuatorId id, S32 value );

    /**
     * Sets the speed of an actuator
     *
     * @param speed Angular velocity radian per sec.
     */
    void setActuatorSpeed( ActuatorId id, double speed );

    /**
     * Set min, max and default value of an actuator.
     *
     * @param id The id of the actuator.
     * @param config New actuator configuration.
     */
    void setActuatorConfig( ActuatorId id, const ActuatorConfig &config );

    /**
     * Sets the front steer servo.
     *
     * @param value New servo value.
     */
    void setSteerFront( S32 value );

    /**
     * Sets the back steer servo.
     *
     * @param value New servo value.
     */
    void setSteerBack( S32 value );

    /**
     * Sets the speed.
     *
     * @param value The new speed servo value.
     */
    void setSpeed( S32 value );

    /**
     * Sets the value of the camera pan servo.
     *
     * @param value The new servo value.
     */
    void setCamPan( S32 value );

    /**
     * Sets the value of the camera tilt servo.
     *
     * @param value New servo value.
     */
    void setCamTilt( S32 value );

    /**
     * Gets a servo.
     *
     * @param type Message type.
     */
    S32 getActuator( ActuatorId id ) const { return mCurrentValue[id]; }

    /**
     * Return the minimum actuator value.
     *
     * @param id Id of the actuator.
     * @return Minimum actuator value.
     */
    S32 getActuatorMin( ActuatorId id ) const { return mConfig[id].min; }

    /**
     * Return the maximum actuator value.
     *
     * @param id Id of the actuator.
     * @return Maximum actuator value.
     */
    S32 getActuatorMax( ActuatorId id ) const { return mConfig[id].max; }

    /**
     * Return the actuator value equal to zero.
     *
     * @param id Id of the actuator.
     * @return Zero actuator value.
     */
    S32 getActuatorZero( ActuatorId id ) const { return mConfig[id].zero; }

    /**
     * Returns the turning velocity of the actuator.
     *
     * @param id Id of the actuator.
     */
    double getActuatorSpeed( ActuatorId id ) const { return mConfig[id].speed; }

    /**
     * Returns the configuration of an actuator.
     *
     * @param id The id of the actuator.
     * @return The actuators configuration.
     */
    ActuatorConfig getActuatorConfig( ActuatorId id ) { return mConfig[id]; }

    /**
     * Gets the front steer servo.
     */
    S32 getSteerFront() const;

    /**
     * Gets the back steer servo.
     */
    S32 getSteerBack() const;

    /**
     * Gets the speed.
     */
    S32 getSpeed() const;

    /**
     * Gets the value of the camera pan servo.
     */
    S32 getCamPan() const;

    /**
     * Gets the value of the tilt servo.
     */
    S32 getCamTilt() const;

    /**
     *  Let ramaxx beep. Plug up your ears.
     */
    void beep( const int *beeps ) const;

private:
    /// Connection to the robot.
    RamaxxConnection * mConn;
    /// Current actuator values.
    S32 mCurrentValue[ACTUATOR_COUNT];
    /// Actuator configurations
    ActuatorConfig mConfig[ACTUATOR_COUNT];
};

#endif /* ACTUATORS_H_ */
