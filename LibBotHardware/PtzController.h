/*
 * PtzController.h
 *
 *  Created on: Feb 20, 2010
 *      Author: marks
 */

#ifndef PTZCONTROLLER_H
#define PTZCONTROLLER_H

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

// Workspace
#include <Stopwatch.h>

// Project
#include "Actuators.h"
#include "RobotSensors.h"
#include "DataListener.h"

///////////////////////////////////////////////////////////////////////////////
// DEFINITIONS
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// DECLARATIONS
///////////////////////////////////////////////////////////////////////////////

/**
 * Holds all necessary configuration data.
 */
struct PtzCalibration {
    bool panMovable;
    double panAngle;
    double panDegToServo;
    bool tiltMovable;
    double tiltAngle;
    double tiltDegToServo;
};

/**
 * Controls the pan/tilt unit. Offers methods to set and to get the
 * current pan/tilt angles.
 */
class PtzController : public DataListener, public DataOwner {
public:
    /**
     * Contructor.
     *
     * @param act The robots actuators.
     * @param sensors All sensors.
     */
    PtzController( Actuators * act, RobotSensors * sensors );

    /**
     * Sends the current servo values to the microcontroller (if necessary).
     * Call this method at least every 20 ms!
     */
    void update();

    /**
     * Sets the pan/tilt angles in degree.
     *
     * @param pan Pan angle in degree.
     * @param tilt Tilt angle in degree.
     */
    void setPanTiltDegree( const float &pan, const float &tilt );

    /**
     * Sets the pan/tilt angles in radian.
     *
     * @param pan Pan angle in radian.
     * @param tilt Tilt angle in radian.
     */
    void setPanTiltRad( const float &pan, const float &tilt );

    /**
     * Sets the pan angle.
     *
     * @param pan The new pan angle in radian.
     */
    void setPanRad( const float &pan );

    /**
     * Sets the tilt angle.
     *
     * @param tilt The new tilt angle in radian.
     */
    void setTiltRad( const float &tilt );

    /**
     * Set the tilt angle to the given constant value. Stop sending commands
     * to the tilt servo.
     *
     * @param tilt The new tilt angle [rad].
     */
    void setFixedTilt( const float &tilt );

    /**
     * Set the pan angle to the given constant value. Stop sending commands
     * to the pan servo.
     *
     * @param pan The new pan angle [rad].
     */
    void setFixedPan( const float &pan );

    /**
     * Returns the pan angle.
     *
     * @param The current pan angle in radian. If there is no ADC that measures
     * the current pan angle, the returned value equals the last requested pan angle.
     * The true pan angle may differ from this value if the ptz unit is still
     * aproaching the requested position.
     */
    float getPanRad() const;

    /**
     * Returns the tilt angle.
     *
     * @param The current tilt angle in radian. If there is no ADC that measures
     * the current tilt angle, the returned value equals the last requested tilt angle.
     * The true angle may differ from this value if the ptz unit is still
     * aproaching the requested position.
     */
    float getTiltRad() const;

    /**
     * Sets the ptz position to 0,0.
     */
    void resetPosition();

    /**
     * Sets the ptz unit calibration data.
     *
     * @param calib The new calibration data.
     */
    void setCalibration( const PtzCalibration &calib );

    /**
     * Adjusts the pan servo zero value.
     *
     * @param delta Pan servo zero position offset.
     */
    void adjustPanServoZero( int delta );

    /**
     * Adjusts the tilt servo zero value.
     *
     * @param delta Tilt servo zero position offset.
     */
    void adjustTiltServoZero( int delta );

    /* Inherited from DataListener */
    void dataChanged( const DataChangeEvent &event );

private:
    /** Actuator id */
    Actuators::ActuatorId mPanId;
    Actuators::ActuatorId mTiltId;
    /** Current calibration data. */
    PtzCalibration  mCalibration;
    /** The robots actuators. */
    Actuators *     mActuators;
    /** All sensors of the robot. */
    RobotSensors *  mSensors;
    /** Pan/tilt angle in radian as measured by the ADC sensor or as set via setFixed(...) . */
    float mPan, mTilt;
    /** Pan/tilt fixed? */
    bool mFixedPan, mFixedTilt;
    /** Requested pan/tilt angle in radian. */
    float mPanRequest, mTiltRequest;
    /** Used to send the current servo valuies every 20 ms. */
    Stopwatch mUpdateStopwatch;
};

#endif // PTZCONTROLLER_H
