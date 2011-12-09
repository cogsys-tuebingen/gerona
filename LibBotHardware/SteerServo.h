/*
 * SteerServo.h
 *
 *  Created on: Nov 19, 2009
 *      Author: marks
 */

#ifndef STEERSERVO_H_
#define STEERSERVO_H_

///////////////////////////////////////////////////////////////////////////////
// FORWARD DECLARATIONS
///////////////////////////////////////////////////////////////////////////////

class SteerServo;

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

// C/C++
#include <string>

// Project
#include "DataListener.h"
#include "Voltage.h"
#include "Actuators.h"
#include "RobotSensors.h"
#include "LogAdapter.h"
#include "SteerAngleController.h"

// Workspace
#include "Stopwatch.h"

///////////////////////////////////////////////////////////////////////////////
// DEFINITIONS
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// DECLARATIONS
///////////////////////////////////////////////////////////////////////////////

/**
 * Contains all necessary calibration values.
 */
struct SteerServoConfig {
    /**
     * Creates a default steer servo configuration.
     */
    SteerServoConfig() :
            updateIntervalMs( 50 ),
            degToServo( 30.0 ),
            useHall( false ),
            enableCtrl( false ),
            ctrlKp( 400 ),
            ctrlKi( 600 )
    {}

    /// Time between two updatesd msec
    unsigned int updateIntervalMs;
    /// Conversion from degree to servo value
    double degToServo;
    /// Hall sensor #1 calibration values (polynomial fit function, lowest order coefficient first)
    vector<double> hallCoefficients1;
    /// Hall sensor #2 calibration values (polynomial fit function, lowest order coefficient first)
    vector<double> hallCoefficients2;
    /// Use the hall sensors to calculate the current steering angle?
    bool useHall;
    /// Enable steer angle controller? Needs enabled and calibrated hall sensors!
    bool enableCtrl;
    /// Steer angle controller proportional gain
    double ctrlKp;
    /// Steer angle controller integral gain
    double ctrlKi;
};

/**
 * Represents a steer servo.
 */
class SteerServo : public DataListener {

public:
    /**
     * Constructor.
     *
     * @param pos The position of the steering.
     * @param actuators The robots actuators.
     * @param sensors The robots sensors.
     */
    SteerServo( Actuators * actuators, RobotSensors * sensors );

    /**
     * Destructor.
     */
    ~SteerServo();

    /**
     * Sets the steering angle.
     *
     * @param angle The steering angle in degrees.
     */
    void setAngleDeg( float angle );

    /**
     * Returns the steering angle.
     *
     * @return The steering angle.
     */
    float getAngleDeg();

    /**
     * Sets the steer servo calibration.
     *
     * @param calib The calibration data.
     */
    void setConfig( const SteerServoConfig &calib );

    /**
     * Return a copy of the current configuration.
     *
     * @return The current configuration object.
     */
    SteerServoConfig getConfig() const { return mConfig; }

    /**
     * Adjusts the servo zero degree value.
     *
     * @param delta Servo zero value offset.
     */
    void adjustServoZero( int delta );

    /**
     * Returns if the steering angle is calculated using the output
     * of the hall sensors.
     *
     * @return False if the hall sensor ADC returns an error or if
     *      the hall sensors are disabled. True otherwise.
     */
    bool usesHallSensors() const { return mConfig.useHall && mHallOk; }

    /**
     * Used to send a set steer servo command every n msec.
     * Call this method at least every 20 msec!
     */
    void update();

    /**
     * Called if a voltage sensor has new servo position data.
     */
    void dataChanged( const DataChangeEvent &event );

protected:

    /**
     * Return the output of the hall sensors.
     *
     * @param sensor ADC measuring the output of the hall sensors.
     * @param outVoltage1 Output of hall sensor #1 [V]
     * @param outVoltage2 Output of hall sensor #2 [V]
     */
    virtual void getHallVoltages( Voltage * sensor, double &outVoltage1, double &outVoltage2 ) = 0;

    /**
     * Computes the servo value for a given steering angle.
     *
     * @param angleDeg The steering angle in decimal degree.
     *
     * @return The servo value.
     */
    virtual S32 angleToServo( const float angleDeg ) const = 0;

    /**
     * Sets the servo value.
     *
     * @param value The value.
     */
    virtual void setValue( const S32 value ) = 0;

    /**
     * Sets the servo value which adjusts the steering angle to 0 degree.
     *
     * @param midValue The servo value.
     */
    virtual void setServoZero( const S32 midValue ) = 0;

    /**
     * Returns the servo value which equals zero degree steering angle.
     *
     * @return The servo value.
     */
    virtual S32 getServoZero() const = 0;

    /// Holds all necessary configuration data
    SteerServoConfig mConfig;
    /// The robots actuators.
    Actuators * mActuators;
    /// Steer angle controller object
    SteerAngleController mCtrl;

private:

    double calcHallAngle( const double v1, const double v2 ) const;

    /// Current steering angle in degree.
    float mAngleDeg;
    /// Requested steering angle.
    float mTargetAngleDeg;
    /// The ADC that measures the output of the hall sensors
    Voltage * mVoltage;
    /// Used to send a set servo command every n msec
    Stopwatch mUpdateTimer;
    /// Time since last command
    Stopwatch mLastCmdTimer;
    /// Flag if the Hall sensors report valid data
    bool mHallOk;
};

class FrontSteerServo : public SteerServo {
public:

    FrontSteerServo( Actuators * actuators, RobotSensors * sensors );

    /* Inherited from SteerServo */
    S32 angleToServo( const float angleDeg ) const;

    /* Inherited from SteerServo */
    void setValue( const S32 value );

    /* Inherited from SteerServo */
    void setServoZero( const S32 midValue );

    /* Inherited from SteerServo */
    S32 getServoZero() const;

    /* Inherited from SteerServo */
    void getHallVoltages( Voltage * sensor, double &outVoltage1, double &outVoltage2 );
};

class BackSteerServo : public SteerServo {
public:

    BackSteerServo( Actuators * actuators, RobotSensors * sensors );

    /* Inherited from SteerServo */
    S32 angleToServo( const float angleDeg ) const;

    /* Inherited from SteerServo */
    void setValue( const S32 value );

    /* Inherited from SteerServo */
    void setServoZero( const S32 midValue );

    /* Inherited from SteerServo */
    S32 getServoZero() const;

    /* Inherited from SteerServo */
    void getHallVoltages( Voltage * sensor, double &outVoltage1, double &outVoltage2 );
};

#endif /* STEERSERVO_H_ */
