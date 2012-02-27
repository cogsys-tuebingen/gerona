/*
 * Actuators.h
 *
 *  Created on: Feb 22, 2010
 *      Author: Marks
 */

#ifndef ACTUATORS_H_
#define ACTUATORS_H_

///////////////////////////////////////////////////////////////////////////////
// I N C L U D E S
///////////////////////////////////////////////////////////////////////////////

// C/C++
#include <sys/time.h>

// Workspace
#include <Global.h>

// Project
#include "UsbConn.h"
#include "QuMessage.h"

///////////////////////////////////////////////////////////////////////////////
// D E F I N I T I O N S
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// D E C L A R A T I O N S
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
     * Default turning velocity in radian per sec. NOTE: This value may be used to
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
 * Represents the present status of an actuator and holds the configuration.
 */
struct ActuatorStatus {

    /// Current configuration
    ActuatorConfig config;

    /// Requested position (won't be equal to the present position in many cases!)
    S32 requestedPosition;

    /// Requested actuator speed in rad/sec
    double requestedSpeed;

    /**
     * Present position as reported by the servo. Alway zero if the actuator doesn't
     * report it's current position.
     */
    S32 presentPosition;

    /**
     * True if the actuator reported it's position once. Currently we are assuming
     * that the actuator won't stop to report it's position if it does not report an
     * error.
     */
    bool hasPositionFeedback;

    /**
     * Time stamp of last position feedback. Won't hold something usefull if
     * there is no position feedback.
     */
    timeval feedbackStamp;

    /// Flag if the actuator reported an error
    bool error;
};

/**
 * Represents the robots actuators and offers methods to set and get the present
 * position.
 */
class Actuators : public QuMsgHandler {
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
     * Set the requested actuator position.
     *
     * @param id Id of the actuator.
     * @param value New actuator position.
     */
    void setPosition( ActuatorId id, S32 value );

    /**
     * Set the requested angular velocity. Currently this affects only AX12 servos. It is
     * not possible to change the angular velocity on an simple PWM servo at the present state.
     *
     * @param id Id of the actuator.
     * @param speed Requested angular velocity rad/sec.
     */
    void setMovingSpeed( ActuatorId id, double speed );

    /**
     * Get the requested actuator position. Note: The true actuator
     * position might differ from this value!
     *
     * @param id Id of the actuator.
     * @return The requested actuator position.
     */
    S32 getRequestedPosition( ActuatorId id ) const { return mActuStatus[id].requestedPosition; }

    /**
     * Returns the actuator position as reported by the actuator itself. The returned value
     * is meaningless if the actuator doesn't report its current position (like simple PWM servos).
     *
     * @param id Id of the actuator.
     * @return The present position.
     */
    S32 getPresentPosition( ActuatorId id ) const { return mActuStatus[id].presentPosition; }

    /**
     * Check if the selected actuator reports its present position (like Dynamixel AX12 servos).
     *
     * @return True if the actuator reported its position once. False otherwise.
     */
    bool hasPositionFeedback( ActuatorId id ) const { return mActuStatus[id].hasPositionFeedback; }

    /**
     * Return the time stamp of the latest position feedback.
     *
     * @param id Id of the actuator.
     * @param msec Milliseconds since the last position feedback was received.
     *
     * @return True if the actuator reported it's position once. False otherwise.
     * Don't use the data in that case.
     */
    bool getPositionFeedbackStamp( const ActuatorId id, int &msec ) const;

    /**
     * Return the minimum actuator value. Note: The actual minimum value might
     * differ from the return value since the Avr32 handles the actuator limits.
     *
     * @param id Id of the actuator.
     * @return Minimum actuator value.
     */
    S32 getMin( ActuatorId id ) const { return mActuStatus[id].config.min; }

    /**
     * Return the maximum actuator value. Note: The actual maximum value might
     * differ from the return value since the Avr32 handles the actuator limits.
     *
     * @param id Id of the actuator.
     * @return Maximum actuator value.
     */
    S32 getMax( ActuatorId id ) const { return mActuStatus[id].config.max; }

    /**
     * Return the actuator value equal to zero.
     *
     * @param id Id of the actuator.
     * @return Zero actuator value.
     */
    S32 getZero( ActuatorId id ) const { return mActuStatus[id].config.zero; }

    /**
     * Returns the default turning velocity of the actuator. Note: The true
     * velocity might differ from the returned value even if the actuator speed
     * has not been changed since simple PWM servos do not control their turning velocity.
     *
     * @param id Id of the actuator.
     * @return The default turning velocity in rad/sec.
     */
    double getDefaultSpeed( ActuatorId id ) const { return mActuStatus[id].config.speed; }

    /**
     * Returns the configuration of an actuator.
     *
     * @param id The id of the actuator.
     * @return The actuators configuration.
     */
    ActuatorConfig getConfig( ActuatorId id ) { return mActuStatus[id].config; }

    /**
     *  Let ramaxx beep. Plug up your ears.
     *
     * @param beeps Beep array. Zero terminated!
     */
    void beep( const int *beeps ) const;

    /* Inherited from QuMsgHandler */
    void processQuMessage( const QuMessage &msg );

/*---- Some getters & setters ----*/

    /**
     * Gets the requested front steer servo value.
     */
    S32 getRequestedSteerFront() const;

    /**
     * Gets the requested back steer servo value.
     */
    S32 getRequestedSteerBack() const;

    /**
     * Gets the requested speed value.
     */
    S32 getRequestedSpeed() const;

    /**
     * Gets the requested value of the camera pan servo.
     */
    S32 getRequestedCamPan() const;

    /**
     * Gets the requested value of the tilt servo.
     */
    S32 getRequestedCamTilt() const;

    /**
     * Set an actuator config.
     *
     * @param id The id of the actuator.
     * @param config New actuator configuration.
     */
    void setConfig( ActuatorId id, const ActuatorConfig &config );

    /**
     * Sets the speed of an actuator
     *
     * @param speed Angular velocity radian per sec.
     */
    void setSpeed( ActuatorId id, double speed );

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

private:
    /// Connection to the robot.
    RamaxxConnection * mConn;

    /// Objects representing the actuators
    ActuatorStatus mActuStatus[ACTUATOR_COUNT];
};

#endif /* ACTUATORS_H_ */
