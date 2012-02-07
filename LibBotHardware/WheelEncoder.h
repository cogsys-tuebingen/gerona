/*
 * WheelEncoder.h
 *
 *  Created on: Jan 19, 2010
 *      Author: marks
 */

#ifndef WHEELENCODER_H_
#define WHEELENCODER_H_

///////////////////////////////////////////////////////////////////////////////
// FORWARD DECLARATIONS
///////////////////////////////////////////////////////////////////////////////

class WheelEncoder;

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

// C/C++
#include <string>

// Workspace
#include <Global.h>
#include <Stopwatch.h>

// Projects
#include "UsbConn.h"
#include "QuMessage.h"
#include "DataListener.h"
#include "Sensor.h"

///////////////////////////////////////////////////////////////////////////////
// DEFINITIONS
///////////////////////////////////////////////////////////////////////////////

// Number of connected sensors
#define WHEELENC_SENSOR_COUNT 2

// Sensor indices
#define WHEELENC_LEFT 0
#define WHEELENC_RIGHT 1

///////////////////////////////////////////////////////////////////////////////
// DECLARATIONS
///////////////////////////////////////////////////////////////////////////////

/**
 * Holds all necessary calibration data for one sensor.
 */
struct WheelEncoderCalib {
    /// Use the data from this sensor?
    bool enable;
    /// Scale factor encoder ticks to meter (forward).
    double scaleFwd;
    /// Scale factor encoder ticks to meter (backward).
    double scaleBwd;
};

/**
 * Represents a wheel encoder, one object represents two actual sensors (left and right).
 */
class WheelEncoder : public Sensor, public QuMsgHandler {
public:

    /**
     * Default contructor.
     *
     * @param conn The connection to the robot.
     */
    WheelEncoder( RamaxxConnection * conn );

    /**
     * Returns latest reported distance of the left sensor.
     *
     * @return The latest reported distance of the left sensor [m].
     */
    double getLeftDistance() const { return mDist[WHEELENC_LEFT]; }

    /**
     * Returns latest reported distance of the right sensor.
     *
     * @return The latest reported distance of the right sensor [m].
     */
    double getRightDistance() const { return mDist[WHEELENC_RIGHT]; }

    /**
     * Returns distance sum of the left sensor.
     *
     * @return The latest reported distance of the left sensor [m].
     */
    double getLeftDistanceSum() const { return mDistSum[WHEELENC_LEFT]; }

    /**
     * Returns distance sum of the right sensor.
     *
     * @return The latest reported distance of the right sensor [m].
     */
    double getRightDistanceSum() const { return mDistSum[WHEELENC_RIGHT]; }

    /**
     * Returns the sum over all reported ticks (left encoder).
     *
     * @return Sum over all reported ticks.
     */
    int getLeftTickSum() const { return mTickSum[WHEELENC_LEFT]; }

    /**
     * Returns the sum over all reported ticks (right encoder).
     *
     * @return Sum over all ticks.
     */
    int getRightTickSum() const { return mTickSum[WHEELENC_RIGHT]; }

    /**
     * Is the left sensor enabled?
     *
     * @return True if the sensor is enabled, false otherwise.
     */
    bool isLeftEnabled() const { return isEnabled( WHEELENC_LEFT ); }

    /**
     * Is the right sensor enabled.
     *
     * @return True if the right sensor is enabled, false otherwise.
     */
    bool isRightEnabled() const { return isEnabled( WHEELENC_RIGHT ); }
    
    /**
     * Returns the time between the two last updates.
     *
     * @return
     *     Time between the two last updates [s].
     */
    double getTimeDifference() const { return mTimeDiff; }

    /**
     * Sets the calibration of the left sensor.
     *
     * @param calib The new calibration data.
     */
    void setLeftCalib( const WheelEncoderCalib &calib ) { setCalib( calib, 0 ); }

    /**
     * Sets the calibration of the right sensor.
     *
     * @param calib The new calibration data.
     */
    void setRightCalib( const WheelEncoderCalib &calib ) { setCalib( calib, 1 ); }

    /**
     * Returns the calibration of the left sensor.
     *
     * @return The calibration data.
     */
    WheelEncoderCalib getLeftCalib() const { return mCalib[0]; }

    /**
     * Returns the calibration of the right sensor.
     *
     * @return The calibration data.
     */
    WheelEncoderCalib getRightCalib() const { return mCalib[1]; }

    /* Inherited from QuMsgHandler */
    void processQuMessage( const QuMessage &msg );

    /* Inherited from Sensor */
    void diagnosis( int timeDiff );

private:
    /**
     * Returns the last reported distance of encoder with index i.
     *
     * @param i The index of the encoder (currently 0 or 1).
     *
     * @return The distance in meters.
     */
    double getDistance( const int i ) const { return mDist[i]; }

    /**
     * Is encoder i enabled?
     *
     * @param i Index of the encoder (0 or 1).
     *
     * @return True if the encoder is enabled.
     */
    bool isEnabled( int i ) const;

    /**
     * Sets the calibration of encoder i.
     *
     * @param calib The new calibration data.
     * @param i Index of the selected encoder.
     */
    void setCalib( const WheelEncoderCalib &calib, int i );

    /// Last reported distance for each encoder in meters.
    double mDist[WHEELENC_SENSOR_COUNT];

    /// Distance sum for each actual sensor.
    double mDistSum[WHEELENC_SENSOR_COUNT];

    /// Distance values used for sensor diagnosis
    double mDiagDist[WHEELENC_SENSOR_COUNT];

    /// Tick sum for each sensor.
    int mTickSum[WHEELENC_SENSOR_COUNT];

    /// Calibration data for each encoder.
    WheelEncoderCalib mCalib[WHEELENC_SENSOR_COUNT];

    /// Enabled flags
    bool mEnabled[WHEELENC_SENSOR_COUNT];

    /// Checked compatibility of configuration and Avr32 firmware?
    bool mEnableChecked;

    /// Time since last update in sec.
    double mTimeDiff;
};

#endif /* WHEELENCODER_H_ */
