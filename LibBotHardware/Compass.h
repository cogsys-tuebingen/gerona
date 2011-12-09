/*
 * Compass.h
 *
 *  Created on: Sep 29, 2009
 *      Author: marks
 */

#ifndef COMPASS_H_
#define COMPASS_H_

///////////////////////////////////////////////////////////////////////////////
// FORWARD DECLARATIONS
///////////////////////////////////////////////////////////////////////////////

class Compass;

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

// C/C++
#include <sys/time.h>
#include <string>

// Project
#include "QuMessage.h"
#include "DataListener.h"
#include "UsbConn.h"
#include "LogAdapter.h"
#include "Sensor.h"

///////////////////////////////////////////////////////////////////////////////
// DEFINITIONS
///////////////////////////////////////////////////////////////////////////////

// Logging ids
const string COMPASS_LOGID_HEADING = "Compass.heading";

///////////////////////////////////////////////////////////////////////////////
// DECLARATIONS
///////////////////////////////////////////////////////////////////////////////

/**
 * Compass interface class.
 */
class Compass : public Sensor {
public:

    /**
     * Returns the heading.
     *
     * @return The heading in radian. 0 equals north, PI / 2 equals east.
     */
    virtual double getHeading() const = 0;

    /**
     * Returns the heading variance.
     *
     * @return The heading variance [rad].
     */
    virtual double getVariance() const = 0;

    /**
     * Sets the value that should be covered as zero heading.
     *
     * @param zeroHeading The zero heading value in radian. Normalized to [0, 2 * PI ).
     */
    virtual void setZeroHeading( double zeroHeading ) = 0;
};

/**
 * Represents a compass connected via I2c.
 */
class I2cCompass : public Compass, public QuMsgHandler, public LogAdapter {

public:

    /**
     * Contructor.
     */
    I2cCompass( RamaxxConnection * conn );

    /**
     * Destructor.
     */
    virtual ~I2cCompass() {}

    /* Inherited from Compass */
    double getHeading() const { return mHeading; }

    /* Inherited from Compass */
    void setZeroHeading( double zeroHeading ) { mZero = zeroHeading; }

    /* Inherited from Compass */
    virtual double getVariance() const { return 0.0685; /*( 15.0*M_PI/180.0 )*( 15.0*M_PI/180.0 )*/ }

    /* Inherited from QuMsgHandler */
    virtual void processQuMessage( const QuMessage &msg );

protected:
    /* Inherited from LogAdapter. */
    void writeLogData( LogCollector * logCollector ) ;

    /* Inherited from LogAdapter. */
    void addLogColumns( LogCollector * logCollector, bool trigger );

private:
    /** Current heading in radian.
     * This value is normalized to [0, 2 * PI ).*/
    double mHeading;
    /** The heading that should be covered as zero. */
    double mZero;
    /** Calibration data counter */
    int mCalibCount;
    /** True if zero heading calibration is done */
    bool mCalibrated;
};

/**
 * Represents a PNI SpacePoint sensor.
 */
class PniCompass : public Compass {
public:

    /** Contructor. */
    PniCompass();

    /** Destructor */
    virtual ~PniCompass() {};

    /* Inherited from Compass */
    double getHeading() const;

    /* Inherited from Compass */
    void setZeroHeading( double zeroHeading );

    /* Inherited from Compass */
    virtual double getVariance() const { return 0.02; /* ca ( 8.0*M_PI/180.0 )*( 8.0*M_PI/180.0 )*/ }

    /* Inherited from QuMsgHandler */
    void processQuMessage( const QuMessage &msg ) { /* Not used */ };

    /**
     * Sets the new data and increments the valid message counter.
     *
     * @param heading New heading direction
     */
    void setNewData( double heading );

private:
    /** Current heading direction [rad] */
    double mHeading;
    /** Zero heading direction [rad] */
    double mZeroHeading;
    /** Number of calibration values */
    int mCalibCounter;
    /** True if zero heading calibration is done. */
    bool mCalibrated;
};

#endif /* COMPASS_H_ */
