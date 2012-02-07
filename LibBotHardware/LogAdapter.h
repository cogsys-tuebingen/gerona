#ifndef LOGADAPTER_H
#define LOGADAPTER_H

////////////////////////////////////////////////////////////////////////////////
// INCLUDES
////////////////////////////////////////////////////////////////////////////////

// Workspace
#include "LogCollector.h"

////////////////////////////////////////////////////////////////////////////////
// DECLARATIONS
////////////////////////////////////////////////////////////////////////////////

/**
 * A universal log adapter. Inherit from this class if you want to log data
 * to a log collector.
 */
class LogAdapter {
public:

    /**
     * Constructor.
     */
    LogAdapter();

    /**
     * Destructor.
     */
    virtual ~LogAdapter() {}

    /**
     * Disable logging.
     */
    void disableLogging() { mLogCollector = NULL; }

    /**
     * Enable logging and initialize the log collector (add all necessary columns).
     *
     * @param logCollector The logging object.
     * @param trigger Trigger the log on data change.
     */
    virtual void enableLogging( LogCollector * logCollector, bool trigger ) ;

protected:
    /**
     * Update the current log data.
     */
    void updateLogValues();

    /**
     * Write all data to the log collector.
     *
     * @param logCollector The log collector.
     */
    virtual void writeLogData( LogCollector * logCollector ) = 0;

    /**
     * Add all necessary columns to a log collector.
     *
     * @param logCollector The log collector.
     * @param trigger Trigger the log on data change.
     */
    virtual void addLogColumns( LogCollector * logCollector, bool trigger ) = 0;

private:
    /** The log collector or NULL if logging is disabled. */
    LogCollector * mLogCollector;
};

#endif // LOGADAPTER_H
