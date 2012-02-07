/*
 * Voltage.h
 *
 *  Created on: Oct 19, 2009
 *      Author: marks
 */

#ifndef VOLTAGE_H_
#define VOLTAGE_H_

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

// C/C++
#include <vector>
#include <sys/time.h>

// Workspace
#include <Global.h>

// Project
#include "QuMessage.h"
#include "DataListener.h"
#include "UsbConn.h"
#include "LogAdapter.h"
#include "Sensor.h"

///////////////////////////////////////////////////////////////////////////////
// DEFINITIONS
///////////////////////////////////////////////////////////////////////////////

// Id of the ADS7828 sensor that measures the steering angles
#define VOLTAGE_ADS7828_STEERID 0x10

///////////////////////////////////////////////////////////////////////////////
// DECLARATIONS
///////////////////////////////////////////////////////////////////////////////

/**
 * Represents a voltage sensor.
 */
class Voltage : public Sensor {
public:

    /** Possible voltage sensor ids. */
    enum VoltageId { STEER, PAN_TILT, AVR };

    /**
     * Constructor.
     *
     * @param id The id of the voltage sensor.
     */
    Voltage( VoltageId id );

    /**
     * Destructor.
     */
    virtual ~Voltage() {}

    /**
     * Returns the id of the voltage sensor.
     *
     * @return The id.
     */
    VoltageId getId() const { return mId; }

    /**
     * Return the number of channels.
     *
     * @return The number of channels.
     */
    size_t getChannelCount() const { return mChannel.size(); }

    /**
     * Returns the time of the last update.
     *
     * @param time The last update time will be written to this parameter.
     */
    void getLastUpdateTime( timeval * time ) const { *time = mLastUpdate; }

    /**
     * Return the conversion result of channel i.
     *
     * @param i Index of the channel.
     *
     * @return Voltage in Volts.
     */
    double getChannel( size_t i ) const { return mChannel[i]; }

protected:

    /** Conversion results ordered by channel number. */
    std::vector<double> mChannel;
    /** Last update time. */
    timeval mLastUpdate;

private:
    /** The id of the voltage sensor. */
    VoltageId mId;
};

/**
 * Dummy sensor. Has always invalid data.
 */
class DummyVoltage : public Voltage {
public:

    /**
     * Constructor.
     */
    DummyVoltage(VoltageId id) : Voltage(id) { setSensorState( ERROR ); }

    /**
     * Destructor.
     */
    virtual ~DummyVoltage() {}
};

/**
 * 12-bit 8 channel (4 differencial channels) I2C analog to digital converter ADS7828.
 */
class Ads7828voltage : public QuMsgHandler, public Voltage {
public:

    /**
     * Constructor.
     *
     * @param id The id of the voltage sensor.
     * @param chipId The id of the chip, equals the i2c address.
     * @param conn The connection to the robot.
     */
    Ads7828voltage( VoltageId id, U8 chipId, RamaxxConnection * conn );

    /**
     * Destructor.
     */
    virtual ~Ads7828voltage() {}

    /* Inherited from QuMsgHandler. */
    void processQuMessage( const QuMessage &msg );

private:
    /** Flag if there is valid data. */
    bool mValid;
    /** I2c address the ads7828 chip. */
    U8 mChipId;
};

/**
 * Avr32 10-bit built in analog to digital converter.
 */
class Avr32Voltage : public QuMsgHandler, public Voltage {
public:
    /**
     * Constructor.
     *
     * @param conn The connection to the robot.
     */
    Avr32Voltage( RamaxxConnection * conn );

    /* Inherited from QuMsgHandler. */
    void processQuMessage( const QuMessage &msg );

private:
    /** Flag if we have valid data. */
    bool mValid;
};

#endif /* VOLTAGE_H_ */
