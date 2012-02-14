/*
 * SpeedController.h
 *
 *  Created on: Nov 19, 2009
 *      Author: marks
 */

#ifndef SPEEDCONTROLLER_H_
#define SPEEDCONTROLLER_H_

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

// C++
#include <time.h>
#include <vector>

// Workspace
#include <Global.h>
#include <Stopwatch.h>
#include <LowPassFilter.h>

// Project
#include "Actuators.h"
#include "DataListener.h"
#include "Odometry.h"

///////////////////////////////////////////////////////////////////////////////
// DECLARATIONS
///////////////////////////////////////////////////////////////////////////////

struct SpeedCtrlCalibration {
    S32 nullForward;
    S32 nullBackward;
    double speedScale;
    double kp;
    double ki;
    double kd;
    double maxSpeed;
    double brakeScale;
    double updateIntervalMs; // update interval in milliseconds
};

/**
 * Speed controller interface class.
 */
class SpeedController : public DataListener, QuMsgHandler {

public:
    /**
     * Contructor.
     *
     * @param conn Connection to the robot.
     */
    SpeedController( RamaxxConnection * conn );

    /**
     * Sets the calibration values.
     *
     * @param calib The new calibration data.
     */
    virtual void setCalibration( const SpeedCtrlCalibration &calib )
        { mCalib = calib; }

    /**
     * Return the current calibration.
     *
     * @return The current calibration.
     */
    virtual SpeedCtrlCalibration getCalibration()
        { return mCalib; }

    /**
     * Set the speed of the robot.
     *
     * @param speed The speed in meters per second; use negative values to drive backwards.
     */
    virtual void setSpeed( float speed ) = 0;

    /**
     * Returns the requested speed in m/s. ATTENTION: The actual speed of the robot
     * may differ from this value.
     *
     * @return The requested speed [m/s].
     */
    virtual float getSpeed() const = 0;

    /**
     * Send the current speed value to the avr32 if necessary.
     *
     * @param immediately: true if update should be done immediately
     *                     false if update should be done only each 0.3 secs
     */
    virtual void update( bool immediately ) = 0;

    /**
     * Turns of controller, i.e. for debugging purposes
     */
    virtual void deactivate() = 0;

    /* Inherited from DataListener */
    virtual void dataChanged( const DataChangeEvent &event ) = 0;

    /* Inherited from DataListener */
    void connectionEstablished();

protected:

    /**
     * Enable/disable the microcontrollers own speed control.
     *
     * @param arg Enable?
     */
    void setAvr32SpeedCtrlEnabled( bool arg );

    /// Current calibration
    SpeedCtrlCalibration mCalib;

private:
    /// Flag if we should enable the Avr32 speed controller
    bool mAvr32Ctrl;

    /// Connection to the robot
    RamaxxConnection * mConn;
};

/**
 * Sends the requested speed to the Avr32 where the speed
 * regulation is done. So this is a "dummy" controller.
 */
class Avr32SpeedController : public SpeedController {
public:

    /**
     * Constructor.
     *
     * @param conn Connection to the robot.
     */
    Avr32SpeedController( RamaxxConnection *conn );

    /* Inherited from SpeedController. */
    void setSpeed( float speed );

    /* Inherited from SpeedController */
    float getSpeed() const { return mTargetSpeed; }

    /* Inherited from SpeedController */
    void update( bool immediately );

   /* Inherited from SpeedController */
    void deactivate();

    /* Inherited from DataListener */
    void dataChanged( const DataChangeEvent &event ) { /* Nothing to do */ }

private:
    /** Requested speed m/s */
    float mTargetSpeed;

    /** Connection to the robot. */
    RamaxxConnection * mConn;

    /** Used to determine the time since the last update. */
    Stopwatch mUpdateTimer;
};

/**
 * Simple speed controller.
 */
class SimpleSpeedController : public SpeedController {
public:
    /**
     * Constructor.
     *
     * @param conn Connection to the robot.
     * @param actuators The robots actuators.
     * @param odo The robots odometry (NULL if not available).
     */
    SimpleSpeedController( RamaxxConnection * conn,
                          Actuators * actuators, Odometry * odo );

    /* Inherited from SpeedController */
    void setCalibration( const SpeedCtrlCalibration &calib );

    /* Inherited from SpeedController. */
    void setSpeed( float speed );

    /* Inherited from SpeedController */
    float getSpeed() const { return mTargetSpeed; }

    /* Inherited from SpeedController */
    void update( bool immediately );

   /* Inherited from SpeedController */
    void deactivate();

    /* Inherited from DataListener */
    void dataChanged( const DataChangeEvent &event );

private:
    /**
     * Sets the servo value by sending a message to the robot.
     *
     * @param value The servo value.
     */
    void setServoValue( U16 value );

    /**
     * Converts a servo value to motor thrust.
     * The unit of thrust corresponds to the speed in meters per second,
     * which the robot achieves when driving on an even ground and when
     * the given thrust is constantly applied for a while.
     *
     * @param speedVal The servo value.
     * @return The corresponding speed in meters per second.
     */
    double servoToThrust( int servoVal ) const;

    /**
     * Converts the given motor thrust value to a servo value.
     * The unit of thrust corresponds to the speed in meters per second,
     * which the robot achieves when driving on an even ground and when
     * the given thrust is constantly applied for a while.
     *
     * @param speed The speed in meters per second.
     * @return The corresponding servo value.
     */
    U16 thrustToServo( float thrust ) const;

    /** The robots actuators. */
    Actuators * mActuators;
    /** Time of the last queued speed servo message. */
    timeval mLastSend;
    /** The speed we should achieve in meters per second. */
    float mTargetSpeed;
    /** The speed the odometry currently reports in meters per second. */
    float mActualSpeed;
    /** The last send speed value in meters per second. */
    float mSpeed;
    /** Minimum servo value to drive forward. */
    U16 mNullForward;
    /** Maximum servo value to drive backwards. */
    U16 mNullBackward;
    /** Speed scale factor. */
    float mSpeedScale;
    /** True if we are in backwards mode. */
    bool mBackwards;
    /** Object doing odometry. */
    Odometry * mOdo;
    /**  stall counter */
    U8 mStall;
    /** true if wheel encoder is present */
    bool mWheelEncoder;
    /** the controller only sends servo commands when active */
    bool mActive;
    /** needed by the current solution of the backward problem */
    U8 mCycler;
    /** roughly represents the friction on the wheels, directly resulting in more
     * power given to the evx
     */
    float mControl;
};

#endif /* SPEEDCONTROLLER_H_ */
