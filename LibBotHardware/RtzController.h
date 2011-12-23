/*
 * PtzController.h
 *
 *  Created on: Nov, 2011
 *      Author: marks
 */

#ifndef RTZCONTROLLER_H
#define RTZCONTROLLER_H

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

// Workspace
#include <Stopwatch.h>

// Project
#include "Actuators.h"
#include "QuMessage.h"

///////////////////////////////////////////////////////////////////////////////
// DECLARATIONS
///////////////////////////////////////////////////////////////////////////////

/**
 * Holds all necessary configuration data.
 */
struct RtzCalibration {
    double rollDegToServo;
    double tiltDegToServo;
};

/**
 * Controls the laser roll/tilt unit. Offers methods to set and to get the
 * current roll/tilt angles.
 */
class RtzController {
public:

  /**
     * Contructor.
     *
     * @param act The robots actuators.
     */
    RtzController( Actuators * act );

    /**
     * Sends the current servo values to the microcontroller (if necessary).
     * Call this method at least every 20 ms!
     */
    void update();

    /**
     * Set the angular velocity of the roll/tilt actuators.
     *
     * @param radPerSec Angular velocity rad/sec. Set this parameter to zero
     *      for maximum speed.
     */
    void setMovingSpeed( double radPerSec );

    /**
     * Sets the roll/tilt angles in degree.
     *
     * @param roll Roll angle in degree.
     * @param tilt Tilt angle in degree.
     */
    void setRollTiltDegree( const double &roll, const double &tilt );

    /**
     * Sets the roll/tilt angles in radian.
     *
     * @param roll Roll angle in radian.
     * @param tilt Tilt angle in radian.
     */
    void setRollTiltRad( const double &roll, const double &tilt );

    /**
     * Sets the roll angle.
     *
     * @param roll The new roll angle in radian.
     */
    void setRollRad( const double &roll );

    /**
     * Sets the tilt angle.
     *
     * @param tilt The new tilt angle in radian.
     */
    void setTiltRad( const double &tilt );

    /**
     * Returns the roll angle.
     *
     * @param The estimated roll angle in radian. The true roll angle
     * may differ from this value if the rtz unit is still
     * aproaching the requested position.
     */
    double getRollRad() const;

    /**
     * Returns the tilt angle.
     *
     * @param The current tilt angle in radian. The true angle may
     * differ from this value if the rtz unit is still
     * aproaching the requested position.
     */
    double getTiltRad() const;

    /**
     * Sets the rtz position to 0,0.
     */
    void resetPosition();

    /**
     * Sets the rtz unit calibration data.
     *
     * @param calib The new calibration data.
     */
    void setCalibration( const RtzCalibration &calib );

private:
    /// Actuator id
    Actuators::ActuatorId mRollId;
    Actuators::ActuatorId mTiltId;
    /// Current calibration data.
    RtzCalibration  mCalib;
    /// The robots actuators.
    Actuators *     mActuators;
    /// Requested roll/tilt angle in radian.
    double mRollRequest, mTiltRequest;
    /// Reported roll/tilt angles in radian.
    double mReportedRoll, mReportedTilt;
    /// Flag if there was a new request
    bool mNewRequest;
    /// Flag if we received at least one position message from the Avr32
    bool mReceivedPosMsg;
    /// Used to send the current servo values every n ms.
    Stopwatch mUpdateStopwatch;
};

#endif // RTZCONTROLLER_H
