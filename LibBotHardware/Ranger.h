#ifndef RANGER_CPP
#define RANGER_CPP

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

// C/C++
#include <vector>
#include <map>
#include <string>

// Workspace
#include <Global.h>

// Project
#include "DataListener.h"
#include "QuMessage.h"
#include "UsbConn.h"
#include "LogAdapter.h"
#include "Sensor.h"

///////////////////////////////////////////////////////////////////////////////
// DECLARATIONS
///////////////////////////////////////////////////////////////////////////////

// TODO Add a ranger proxy to avoid multiple calls of processQuMessage

/**
 * Represents the position of a range sensor.
 */
struct RangerPosition {

    /**
     * All member default to zero.
     */
    RangerPosition() {
        x = y  = z = a = 0.0f;
    }

    /**
     * Initialize all member
     */
    RangerPosition( float x_, float y_, float z_, float a_ )
        : x( x_ ), y( y_ ), z( z_ ), a( a_ ) {}

    /// x coordinate [m]
    float x;

    /// y coordinate [m]
    float y;

    /// z coordinate [m]
    float z;

    /// Viewing angle [rad]
    float a;

};

/**
 * Base class of all one dimensional range sensors (e.g. sonar sensors). Provides
 * common functions.
 */
class Ranger : public Sensor, public LogAdapter {
public:

    /**
     * Possible range sensor types.
     */
    enum RangerType { SONAR, IR };

    /**
     * Constructor. The number of avaliable ranges is set to zero.
     *
     * @param type The type of the range sensor.
     * @param pos Position of the range sensor (robot coordinates).
     */
    Ranger( RangerType type, RangerPosition pos );

    /**
     * Destructor.
     */
    virtual ~Ranger() {}

    /**
     * Get the position of the range sensor.
     *
     * @param pos The position of ranger will be written to this parameter.
     */
    void getRangerPosition(  RangerPosition * pos ) const { *pos = mPos; }

    /**
     * Returns the current range measurement.
     *.
     * @return The current range [m].
     */
    float getRange() const { return mRange; }

    /**
     * @brief Returns the range measurement in robot coordinates.
     * Call getRange() to get the raw distance measurement. Check if
     * this value is valid.
     *
     * @param x x coordinate
     * @param y y coordinate
     * @param z z coordinate
     */
    void getRange( float &x, float &y, float &z ) const;

    /**
     * Returns the type of the range sensor.
     *
     * @return The type of the range sensor.
     */
    RangerType getRangerType() const { return mType; }

protected:

    /**
     * Sets the range. Does not notify the data listeners!
     *
     * @param newRange The new range [m].
     */
    void setRange( float newRange, bool valid );

    /* Inherited from LogAdapter. */
    virtual void writeLogData( LogCollector * logCollector ) = 0;

    /* Inherited from LogAdapter. */
    virtual void addLogColumns( LogCollector * logCollector, bool trigger ) = 0;

    /** Position of the sensor (robot coordinates). */
    RangerPosition mPos;

private:
    /** Current ranger data [m]. */
    float mRange;
    /** Type of the range sensor. */
    RangerType mType;
};

/**
 * Represent a sonar ranger.
 */
class SonarRanger : public Ranger, public QuMsgHandler {
public:

    /**
     * Constructor.
     *
     * @param i2cAddress The i2c address of the sensor. Used to associate this
     *  this object with incoming sonar data  messages.
     * @param pos The position of the sensor (robot coordinates).
     * @param conn The connection to the robot.
     */
    SonarRanger( U8 i2cAddress, RamaxxConnection * conn, RangerPosition pos = RangerPosition());

    /**
     * Sets the position of the ranger.
     *
     * @param pos The sonar ranger position.
     */
    void setPosition( const RangerPosition &pos );


    /* Inherited from QuMsgHandler */
    void processQuMessage( const QuMessage &msg );

protected:
    /* Inherited from Ranger. */
    void writeLogData( LogCollector * logCollector );

    /* Inherited from Ranger. */
    void addLogColumns( LogCollector * logCollector, bool trigger );

private:
    /** I2c address of the represented sensor. */
    U8 mAddress;
};

/**
 * Represent a ir ranger.
 */
class IrRanger : public Ranger, public QuMsgHandler {
public:

    /**
     * Constructor. Number of available sensors defaults to zero.
     *
     * @param rangerId Id the IR ranger (correlates with the ADC channel).
     * @param pos Position of the sensor.
     * @param conn Connection to the robot.
     */
    IrRanger( U8 rangerId, RangerPosition pos, RamaxxConnection * conn );

    /* Inherited from QuMsgHandler */
    void processQuMessage( const QuMessage &msg );

protected:
    /* Inherited from Ranger. */
    void writeLogData( LogCollector * logCollector );

    /* Inherited from Ranger. */
    void addLogColumns( LogCollector * logCollector, bool trigger );

private:
    /** Id of the IR ranger (correlates with the ADC channel number). */
    int mId;
};

#endif // RANGER_CPP
