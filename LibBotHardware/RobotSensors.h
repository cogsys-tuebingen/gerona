/*
 * Sensors.h
 *
 *  Created on: Feb 20, 2010
 *      Author: marks
 */

#ifndef SENSORS_H_
#define SENSORS_H_

///////////////////////////////////////////////////////////////////////////////
// FORWARD DECLARATIONS
///////////////////////////////////////////////////////////////////////////////

class RobotSensors;
class Ramaxx;

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

// C/C++
#include <vector>

// Workspace
#include <Stopwatch.h>

// Project
#include "Compass.h"
#include "WheelEncoder.h"
#include "UsbConn.h"
#include "Voltage.h"
#include "Ranger.h"
#include "Adns.h"

///////////////////////////////////////////////////////////////////////////////
// DECLARATIONS
///////////////////////////////////////////////////////////////////////////////

// Typedefs
typedef std::vector<Compass*> CompassVector;
typedef std::vector<WheelEncoder*> WheelEncVector;
typedef std::vector<Voltage*> VoltageVector;
typedef std::vector<SonarRanger*> SonarVector;
typedef std::vector<IrRanger*> IrVector;
typedef std::vector<RangerPosition> RangerPositionVector;
typedef std::vector<AdnsSensor*> AdnsSensorVector;

// Classes

/**
 * The robots sensors.
 */
class RobotSensors {
public:

    /**
     * Constructor.
     *
     * @param conn The connection to the robot.
     */
    RobotSensors( RamaxxConnection * conn );

    /**
     * Destructor.
     */
    virtual ~RobotSensors();

    /**
     * Run sensor diagnosis.
     *
     * @param timeDiff Time since last diagnosis [msec]
     */
    void runDiagnosis( int timeDiff );

    /**
     * Returns compass sensor i. Use this function to iterate over all
     * available compass senors.
     *
     * @param i Index of the compass sensor.
     * @return Compass sensor i.
     */
    Compass* getCompass( int i = 0 ) { return mCompassVec[i]; }

    /**
     * Returns the PNI spacepoint compass.
     * @attention Check if this sensor is available!
     *
     * @return The PNI spacepoint compass or NULL if the sensor is not available.
     */
    PniCompass* getPniCompass() { return mPniCompass; }

    /**
     * Returns if the PNI spacepoint compass is available.
     *
     * @return True if there is a PNI compass.
     */
    bool hasPniCompass() const { return mPniCompass != NULL; }

    /**
     * Returns the "builtin" i2c compass.
     * @attention Check if this sensor is available!
     *
     * @return The i2c compass or NULL if the sensor is not available.
     */
    I2cCompass* getI2cCompass() { return mI2cCompass; }

    /**
     * Returns if there is a i2c compass available.
     *
     * @return True if there is a i2c compass.
     */
    bool hasI2cCompass() const { return mI2cCompass != NULL; }

    /**
     * Returns the number of available compass sensors.
     *
     * @return Number of compass sensors.
     */
    int getCompassCount() const { return mCompassVec.size(); }

    /**
     * Returns the number of available voltage sensors.
     *
     * @return Number of available voltage sensors.
     */
    int getVoltageCount() const { return mVoltageVec.size(); }

    /**
     * Returns voltage sensor i. Use this function to iterate over
     * all available sensors
     *
     * @param i Index of the voltage sensor.
     * @return Voltage sensor i.
     */
    Voltage* getVoltage( int i ) { return mVoltageVec[i]; }

    /**
     * Returns the voltage sensor that measures the steering angles.
     * @attention Check if this sensor is available!
     *
     * @return The steering angles measuring voltage sensor or NULL
     * if the sensor is not available.
     */
    Voltage* getSteerVoltage() { return mSteerVoltage; }

    /**
     * Returns if there is a steer voltage sensor available.
     *
     * @return True if the sensor is available.
     */
    bool hasSteerVoltage() const { return mSteerVoltage != NULL; }

    /**
     * Returns the voltage sensor that measures the pan/tilt angles.
     * @attention Check if this sensor is available!
     *
     * @return The pan/tilt angles measuring voltage sensor or NULL
     * if the sensor is not available.
     */
    Voltage* getPanTiltVoltage() { return mPtzVoltage; }

    /**
     * Returns if there is a ptz voltage sensor available.
     *
     * @return True if the sensor is available.
     */
    bool hasPtzVoltage() const { return mPtzVoltage != NULL; }

    /**
     * Returns the voltage sensor that represents the AVR32 built-in
     * ADC.
     * @attention Check if this sensor is available!
     *
     * @return The AVR32 built-in analog to digital converter or NULL
     * if the sensor is not available.
     */
    Voltage* getAvrVoltage() { return mAvrVoltage; }

    /**
     * Returns if there is a avr voltage sensor available.
     *
     * @return True if the sensor is available.
     */
    bool hasAvrVoltage() const { return mAvrVoltage != NULL; }

    /**
     * Returns the number of available sonar range sensors.
     *
     * @return The number of available sonar sensors.
     */
    int getSonarRangerCount() const { return mSonarVec.size(); }

    /**
     * Returns sonar ranger i. Use this function to iterate over all
     * available sonar ranger.
     *
     * @param i Index of the sonar ranger.
     * @return Reference to sonar ranger i.
     */
    Ranger* getSonarRanger( int i ) { return mSonarVec[i]; }

    /**
     * Returns the number of available IR range sensors.
     *
     * @return Number of IR ranger.
     */
    int getIrRangerCount() const { return mIrVec.size(); }

    /**
     * Returns IR ranger i.
     *
     * @param i Index of the IR ranger.
     * @return reference to the IR ranger i.
     */
    Ranger* getIrRanger( int i ) { return mIrVec[i]; }

    /**
     * Returns the wheel encoder i. Use this function to iterate
     * over all available wheelencoder.
     *
     * @param i Index of the wheel encoder.
     *
     * @return Wheel encoder i.
     */
    WheelEncoder* getWheelEncoder( int i ) { return mWheelencVec[i]; }

    /**
     * Returns the rear wheelencoder.
     * @attention Check if this sensor is available!
     *
     * @return The rear wheelencoder or NULL if there is no rear wheelencoder.
     */
    WheelEncoder* getRearWheelEncoder() { return mRearWheelenc; }

    /**
     * Check if the rear wheelencoder is available.
     *
     * @return True if the sensor is available.
     */
    bool hasRearWheelEncoder() const { return mRearWheelenc != NULL; }

    /**
     * Returns the number of availabel  wheel encoder.
     *
     * @return The number of wheel encoder.
     */
    int getWheelEncoderCount() const { return mWheelencVec.size(); }

    /**
     * Returns the ADNS flowdometry sensor.
     *
     * @param ADNS flowdometry sensor.
     */
    AdnsSensor* getAdnsSensor() { return mAdns; }

    /**
     * Sets the position of sonar ranger i.
     *
     * @param pos The position of the ranger.
     * @param i Index of the ranger.
     */
    void setSonarRangerPosition( const RangerPosition &pos, int i );

    /**
     * Sets the positions of all sonar rangers.
     *
     * @param pos The sonar ranger positions. pos[0] equals the position
     *  of ranger 0, pos[1] of ranger 1 etc.
     */
    void setSonarRangerPosition( const RangerPositionVector &pos );

private:

    /** Connection to the robot. */
    RamaxxConnection * mConn;

    //// Sensor objects ////

    /** Rear wheel encoder (left and right) */
    WheelEncoder *mRearWheelenc;

    /** I2c compass  ("builtin compass") */
    I2cCompass *mI2cCompass;

    /** PNI spacepoint compass. */
    PniCompass *mPniCompass;

    /** ADC measuring ptz angles */
    Ads7828voltage *mPtzVoltage;

    /** ADC measuring steer angles */
    Ads7828voltage *mSteerVoltage;

    /** Avr32 builtin ADC */
    Avr32Voltage *mAvrVoltage;

    /** Sonar ranger with address E0 */
    SonarRanger *mSonarE0;

    /** Sonar ranger with address E2 */
    SonarRanger *mSonarE2;

    /** Sonar ranger with address E4 */
    SonarRanger *mSonarE4;

    /** Sonar ranger with address E6 */
    SonarRanger *mSonarE6;

    /** Sonar ranger with address E8 */
    SonarRanger *mSonarE8;

    /** Sonar ranger with address EA */
    SonarRanger *mSonarEA;

    /** ADNS 2620 flowdometry */
    AdnsSensor *mAdns;

    //// Sensor vectors ////

    /** Compass sensors. */
    CompassVector mCompassVec;

    /** Wheel encoder. */
    WheelEncVector mWheelencVec;

    /** Voltage sensors. */
    VoltageVector mVoltageVec;

    /** Sonar ranger. */
    SonarVector mSonarVec;

    /** Ir ranger. */
    IrVector mIrVec;

    /** Adns mouse sensors. */
    AdnsSensorVector mAdnsVec;
};

#endif /* SENSORS_H_ */
