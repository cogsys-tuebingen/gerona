/*
 * AdnsOdometry.h
 *
 *  Created on: Sep 14, 2009
 *      Author: marks
 */

#ifndef ADNS_H_
#define ADNS_H_

///////////////////////////////////////////////////////////////////////////////
// FORWARD DELARATIONS
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

// C/C++
#include <vector>
#include <string>

// Workspace
#include <Global.h>

// Project
#include "QuMessage.h"
#include "UsbConn.h"
#include "DataListener.h"

///////////////////////////////////////////////////////////////////////////////
// DEFINITIONS
///////////////////////////////////////////////////////////////////////////////

// General ADNS error numbers

// One sensor reported a wrong status
// Message length: 4 bytes
// Byte 0 = ADNS_ERROR_STATUS
// Byte 1 = Bit 0 to 6: Sensor ID. Bit 7: Set if the sensor is locally connected.
// Byte 2 = ADNS chip number.
// Byte 3 = The reported status.
#define ADNS_ERROR_STATUS               0x00

// Error during the readout of a pixdump (e.g. start of frame not found)
// Message length: 4 bytes
// Byte 0 = ADNS_ERROR_PIXDUMP
// Byte 1 = Bit 0 to 6: Sensor ID. Bit 7: Set if the sensor is locally connected.
// Byte 2 = ADNS chip number.
// Byte 3 = Not used.
#define ADNS_ERROR_PIXDUMP              0x01

// TWI specific error numbers

// General TWI error (e.g. chip is not responding)
// Message length: 4 bytes
// Byte 0 = ADNS_ERROR_TWI
// Byte 1 = Bit 0 to 6: Sensor ID. Bit 7: Set if the sensor is locally connected.
// Byte 2 = Not used.
// Byte 3 = Not used.
#define ADNS_ERROR_TWI                  0x10

// TWI readout error (chip is not ready, reported 0x80)
// Message length: 4 bytes
// Byte 0 = ADNS_ERROR_CHIP_NOT_READY
// Byte 1 = Bit 0 to 6: Sensor ID. Bit 7: Set if the sensor is locally connected.
// Byte 2 = Not used.
// Byte 3 = Not used.
#define ADNS_ERROR_CHIP_NOT_READY       0x11

///////////////////////////////////////////////////////////////////////////////
// DECLARATIONS
///////////////////////////////////////////////////////////////////////////////

class AdnsChip {
public:

    /**
     * Constructor.
     */
    AdnsChip();

    /**
     * Resets all values.
     */
    void reset();

    /**
     * Sets the current values.
     *
     * @param dx Current dx value.
     * @param dy Current dy value.
     * @param squal Current surface quality.
     */
    void set( int dx, int dy, int squal );

    /**
     * Returns the last reported surface quality value.
     *
     * @return The last reported SQUAL.
     */
    int getSqual() const { return mSqual; }

    /**
     * Returns the last reported dx value.
     *
     * @return The last reported dx value.
     */
    int getDx() const { return mDx; }

    /**
     * Returns the last reported dy value.
     *
     * @return The last reported dy value.
     */
    int getDy() const { return mDy; }

    /**
     * Returns the sum over all dx values.
     *
     * @return The sum over all dx values.
     */
    int getDxSum() const { return mDxSum; }

    /**
     * Returns the sum over all dy values.
     *
     * @return The sum over all dy values.
     */
    int getDySum() const { return mDySum; }

private:
    /** Last reported surface quality. */
    int mSqual;
    /** Last reported dx value. */
    int mDx;
    /** Last reported dy value. */
    int mDy;
    /** Sum over all dx values. */
    int mDxSum;
    /** Sum over all dy values. */
    int mDySum;
};

typedef vector<AdnsChip> AdnsChipVector;

class AdnsSensor : public DataOwner, public QuMsgHandler {
public:

    /**
     * Constructor.
     *
     * @param sensorId Id of the ADNS sensor.
     * @param chipCount Number of connected ADNS chips.
     * @param conn The connection to the robot.
     */
    AdnsSensor( U8 sensorId, size_t chipCount, RamaxxConnection * conn );

    /**
     * Returns the number of connected chips.
     *
     * @return Tzhe number of connected chips.
     */
    AdnsChip* getChip( size_t i ) { return &mChips[i]; }

    /**
     * Returns chip i.
     *
     * @param i Index of the chip.
     * @return Chip i.
     */
    size_t getChipCount() const { return mChips.size(); }

    /** Inherited from QuMsgHandler */
    void processQuMessage( const QuMessage &msg );

private:

    /**
     * Process a position message.
     *
     * @param msg The message.
     *
     * @return True if the message was processed.
     */
    bool processPositionMsg( const QuMessage &msg );


    /** All ADNS chips. */
    AdnsChipVector mChips;
    /** Id of the sensor. */
    U8 mSensorId;
};

/**
 * Represents a sensor id. Used to identify a sensor.
 */
class AdnsSensorId {
public:

    /**
     * Constructor.
     *
     * @param sensorId  The id of of the sensor.
     * @param chipNumber    The number of the chip.
     */
    AdnsSensorId( Uint sensorId = 0, Uint chipNumber = 0 );

    /**
     * Set the chip number.
     *
     * @param chipNumber The new chip number.
     */
    void setChipNumber( Uint chipNumber ) { mChipNumber = chipNumber; }

    /**
     * Set the sensor id.
     *
     * @param sensorId The new sensor id.
     */
    void setSensorId( Uint sensorId ) { mSensorId = sensorId; }

    /**
     * @return True if the represented sensor is locally connected. False if
     *      the sensor is connected via TWI.
     */
    bool isLocallyConnected();

    /**
     * @return The number of the chip.
     */
    Uint getChipNumber() const { return mChipNumber; }

    /**
     * @return The sensor id.
     */
    Uint getSensorId() const { return mSensorId; }

    /**
     * Copies the sensor id and the chip number from the given object.
     *
     * @param id
     *      The object holding the new values.
     */
    void set( const AdnsSensorId& id ) {
        mSensorId = id.getSensorId();
        mChipNumber = id.getChipNumber();
    }

private:
    // Member variables
    Uint mSensorId;
    Uint mChipNumber;
};

/**
 * Represents an ADNS error message.
 */
class AdnsError {
public:
    /**
     * Constructor.
     */
    AdnsError( const QuMessage& msg );

    /**
     * Create a string representation of this error message.
     */
    string toString() const;

private:
    // Member variables
    Uint            mErrorId;   // Error id
    AdnsSensorId    mId;        // Id of the sensor
    Uint            mValue;     // Value
};

/**
 * Represents the position data of one ADNS sensor.
 */
class AdnsPositionData {
public:
    /**
     * Constructor.
     */
    AdnsPositionData();

    /**
     * @return The delta x value.
     */
    S16 getDx() const { return mDx; }

    /**
     * @return The delta y value.
     */
    S16 getDy() const { return mDy; }

    /**
     * @return The last measured surface quality value.
     */
    U8 getSq() const { return mSq; }

    /**
     * @return The id of the sensor.
     */
    const AdnsSensorId * getId() const { return &mId; }

    /**
     * Sets all values.
     *
     * @param dx
     *     The delta x value.
     * @param dy
     *     The delta y value.
     * @param sq
     *     The surface quality value.
     */
    void set( AdnsSensorId& id, S16 dx, S16 dy, U8 sq ) {
        mId.set( id );
        mDx = dx;
        mDy = dy;
        mSq = sq;
    }

    /**
     * Checks if this data looks valid.
     *
     * @return False if the data is invalid.
     */
    bool isValid() const;

private:
    // Member variables
    AdnsSensorId    mId;
    S16             mDx;
    S16             mDy;
    U8              mSq;
};

class AdnsListener {
public:

    /**
     * Destructor.
     */
    virtual ~AdnsListener(){;}

    /**
     * Process new position data.
     *
     * @param data  The new position data.
     */
    virtual void processPositionData( AdnsSensorId &id, Uint time, const vector<AdnsPositionData>& data ) = 0;

    /**
     * Process a new error message.
     *
     * @param error The error message.
     */
    virtual void processError( const AdnsError &error ) = 0;

    /**
     * Process new pixdump data.
     *
     * @param id The sensor id.
     * @param data  The pixdump data.
     */
    virtual void processPixdumpData( AdnsSensorId &id, const vector<int> &data ) = 0;

};

class AdnsCommunication : public QuMsgHandler {
public:

    /**
     * Constructor.
     *
     * @param conn Connection to the robot.
     */
    AdnsCommunication( RamaxxConnection * conn );

    /**
     * Process a ADNS message.
     *
     * @param msg The message.
     */
    void processQuMessage( const QuMessage &msg );

    /**
     * Register an ADNS listener.
     *
     * @param lis The new listener.
     */
    void addAdnsListener( AdnsListener * lis );

    /**
     * Remove an ADNS listener.
     *
     * @param lis The listener.
     */
    void removeAdnsListener( AdnsListener * lis );

    void setSensorMode( AdnsSensorId &id, int mode );

private:

    /**
     * Process a ADNS move message. Parse the data and notify all
     * registered listeners.
     *
     * @param msg The message.
     */
    void processPositionMsg( const QuMessage &msg );

    /**
     * Parses a ADNS position message send by the robot.
     *
     * @param msg
     *      The message.
     * @param data
     *      The parsed data will be stored into this variable. The vector will be
     *      resized to match the number of data structs.
     * @param time
     *      The timestamp of the message will be stored into this variable.
     *
     * @return
     *      False if the given message is invalid. True otherwise.
     */
    bool parsePositionMsg( const QuMessage &msg, vector<AdnsPositionData> &data, U32 &time );

    /**
     * Process an ADNS error message. Parse the message and notify all
     * registered listeners.
     *
     * @param The message.
     */
    void processErrorMsg( const QuMessage &msg );

    /**
     * Process a pixdump message and notify all registered listeners.
     *
     * @param msg The message.
     */
    void processDumpMsg( const QuMessage &msg );

    /** All registered listeners. */
    vector<AdnsListener*>       mListeners;
    /** The data send by the ADNS sensors. */
    vector<AdnsPositionData>    mPositionList;
};

#endif /* ADNS_H_ */
