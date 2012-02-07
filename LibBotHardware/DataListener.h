/*
 * DataListener.h
 *
 *  Created on: Feb 20, 2010
 *      Author: marks
 */

#ifndef DATALISTENER_H_
#define DATALISTENER_H_

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

// C/C++
#include <vector>

///////////////////////////////////////////////////////////////////////////////
// DECLARATIONS
///////////////////////////////////////////////////////////////////////////////

// Forward declarations
class DataListener;
class DataOwner;

// Typedefs
typedef std::vector<DataListener*> DataListenerVector;

// Classes

/**
 * Represents a data change event. Holds the event type and the corresponding
 * data owner.
 */
class DataChangeEvent {
public:
    /** Possible event types */
    enum EventType { NEW_DATA, STATE };

    /** The event type */
    EventType eventType;
    /** Data owner that fired this event */
    DataOwner* owner;
};

/**
 * Interface class for all data listeners.
 */
class DataListener {
public:

    /**
     * Destructor.
     */
    virtual ~DataListener() {}

    /**
     * The data of the data owner changed.
     *
     * @param owner The data owner.
     */
    virtual void dataChanged( const DataChangeEvent &event ) = 0;
};

/**
 * Base class of all data owners.
 */
class DataOwner {
public:

    /**
     * Default constructor. Sets the valid data flag to false.
     */
    DataOwner();

    /**
     * Destructor.
     */
    virtual ~DataOwner() {}

    /**
     * Returns if the data owner holds valid data.
     *
     * @return True if there is valid data. False otherwise.
     */
    virtual bool hasValidData() const { return mValidData; }

    /**
     * Register a new data listener.
     *
     * @param listener The new data listener.
     */
    virtual void addDataListener( DataListener * listener );

    /**
     * Remove a data listener.
     *
     * @param listener The data listener to remove.
     */
    virtual void removeDataListener( DataListener * listener );

protected:

    /**
     * Call this method if there is new data available. Notifys all registered listeners.
     */
    virtual void fireDataChanged();

    /**
     * Call this method if the data state changed.
     */
    virtual void fireStateChanged();

    /**
     * Sets the valid data flag. Notifys all listeners if the data state changed.
     *
     * @param valid True if there is valid data.
     */
    virtual void setValidData( bool valid );

private:
    /** All registered listeners. */
    DataListenerVector mDataLis;
    /** Flag if this data owner holds valid data */
    bool mValidData;
};

/**
 * Simple flag to check if the data of a data owner has changed since the last
 * check.
 */
class DataChangeFlag : public DataListener {
public:

    /**
     * Constructor.
     */
    DataChangeFlag();

    /**
     * Sets the data change flag to false.
     */
    void reset() { mChangeFlag = false; }

    /**
     * Returns if the data of the observed data owner has changed since the
     * last reset.
     *
     * @return true if the data has changed, false otherwise.
     */
    bool hasChanged() const { return mChangeFlag; }

    /**
     * Sets the data changed flag.
     *
     * @param changed True if the data changed.
     */
    void setChanged( bool changed ) { mChangeFlag = changed; }

    /* Inherited from DataListener */
    void dataChanged( const DataChangeEvent &event );

private:
    /** true if the data of the observed data owner changed since last reset. */
    bool mChangeFlag;
};
#endif /* DATALISTENER_H_ */
