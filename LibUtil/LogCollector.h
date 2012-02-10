/**
 * @project outdoor
 *
 * General class to collect sensor and other log values ordered by time.
 */

#ifndef LOGCOLLECTOR_H
#define LOGCOLLECTOR_H

////////////////////////////////////////////////////////////////////////////////
// INCLUDES
////////////////////////////////////////////////////////////////////////////////

// C/C++
#include <string>
#include <vector>
#include <map>
#include <list>
#include <iostream>
#include <fstream>
#include <sys/time.h>
#include "Stopwatch.h"
////////////////////////////////////////////////////////////////////////////////
// DECLARATIONS
////////////////////////////////////////////////////////////////////////////////

using namespace std;

// Typedefs and Structs
typedef struct {
    bool   isTrigger;
    double value;
    string description;
    int colNum;
} LogColumn;

typedef map<string,int> LogColumnMap;
typedef vector<LogColumn> ColumnVector;

/**
 * Collect and save log data ordered by time.
 */
class LogCollector {
public:

    /**
     * Constructor. Logging is disabled by default.
     */
    LogCollector();

    /**
     * Destructor. Closes the logfile.
     */
    virtual ~LogCollector();

    /**
     * Add a double column to the log.
     *
     * @param id The id of the column. Each column id should be unique.
     * @param description Descritpion of the column data.
     * @param isTrigger Write the current log data if there is new data in this column.
     *    If set to false, some values might get lost.
     *
     * @return False if id exists already, true otherwise
     */
    bool addColumn( const string& id, const string& description , bool isTrigger = false );

    bool addColumn( const string& id);

    void addColumns(const std::list<string>& ids);

    /**
     * Enable collecting of data.
     *
     * @param fileName Name of the logfile.
     * @param overwrite Overwrite an existing file with the same name?
     *
     * @return False if there was an error, true otherwise.
     */
    bool enable( const string &fileName, bool overwrite = false );

    /**
     * Disable logging. The logfile will be closed.
     */
    void disable();

    /**
     * Adds value to the end of a data column.
     *
     * @param id The id of the column.
     * @param value The new value.
     *
     * @return false if id does not exist
     */
    bool addValue( const string& id, double value );

    /**
     * Adds value to the end of a data column.
     *
     * @param id The id of the column.
     * @param value The new value.
     *
     * @return false if id does not exist
     */
    bool addValue( const string& id, int value );

    /**
     * Returns the current log data.
     *
     * @param data The current data will be written to this variable, the data
     *      equals one line in the logfile.
     */
    void getCurrentValues( vector<double> &data ) const;

    /**
     * Write the current log data to the logfile.
     */
    bool writeLogLine();


    bool isEnabled() {return mEnabled;}

    /**
      if called created matlab-style comments starting with %
      */
    void enableMatlab() {mCommentChar='%';}

protected:

    /**
     * Write to the current log data to the given stream. Format is CSV.
     *
     * @param outStream The stream.
     *
     * @return True if there was no error, false otherwise.
     */
    virtual bool writeLine( ofstream* outStream );

    /**
     * Write a column description to the logfile.
     *
     * @param col The data column.
     * @param outStream The data stream.
     *
     * @return True if there was no error, false otherwise.
     */
    virtual bool writeDescription( const LogColumn &col, ofstream * outStream );

    // Member
    /** Flag if logging is enabled. */
    bool          mEnabled;
    /** Maps the column ids to a column. */
    LogColumnMap  mColMap;
    /** Holds the columns in ascending order. */
    ColumnVector  mColData;
    /** Logfile out or NULL if logging is disabled. */
    ofstream *    mOutStream;
    /** Number of columns. */
    int           mColNum;
    /** Logging enable timestamp. */
    timeval       mEnableTime;
    char          mCommentChar;
    Stopwatch     mLogTimer;
};


#endif // LOGCOLLECTOR_H
