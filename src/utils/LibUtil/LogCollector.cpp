
////////////////////////////////////////////////////////////////////////////////
// INCLUDES
////////////////////////////////////////////////////////////////////////////////

// C/C++
#include <stdio.h>
#include <ios>

// Project
#include "LogCollector.h"
#include "Misc.h"

////////////////////////////////////////////////////////////////////////////////
// IMPLEMENTATION
////////////////////////////////////////////////////////////////////////////////

//// class LogCollector ////////////////////////////////////////////////////////

LogCollector::LogCollector() {
    mEnabled = false;
    mOutStream = NULL;
    mColNum = 0;
    mCommentChar='#';
}

LogCollector::~LogCollector() {
    // Close the logfile
    if ( mEnabled ) {
        mOutStream->close();
        delete mOutStream;
    }
}

bool LogCollector::enable( const string &fileName, bool overwrite ) {
    // Disable logging if logging is enabled
    if ( mEnabled ) {
        disable();
    }

    // Check if the logfile exists already
    int numExt = 1;
    char buffer[3];
    string realFileName( fileName );
    while ( Misc::fileExists( realFileName.c_str()) && !overwrite ) {
        // Check if the the buffer is big enought
        if ( numExt > 999 )  {
            return false;
        }

        // Try next number
        sprintf( buffer, "%03d", numExt );
        realFileName = fileName;
        realFileName += string( buffer, 3 );
        numExt++;
    }

    // Open log file
    mOutStream = new ofstream( realFileName.c_str());
    if ( !*mOutStream ) {
        // What ever
        return false;
    }
    mEnabled = true;

    // Set enable time
    gettimeofday( &mEnableTime, 0 );

    // Write date and time
    time_t rawtime;
    time( &rawtime );
    struct tm * timeinfo = localtime( &rawtime );
    *mOutStream << mCommentChar <<"\n";
    *mOutStream << mCommentChar << " Ramaxx logfile.\n";
    *mOutStream << mCommentChar << " Logging started at " << asctime( timeinfo );
    *mOutStream << mCommentChar << " \n";

    // Write descriptions
    *mOutStream << mCommentChar<< " Column 1: Time since loggin was enabled [msec]" << endl;
    ColumnVector::iterator colIter = mColData.begin();
    for ( ; colIter != mColData.end(); colIter++ ) {
        writeDescription( *colIter, mOutStream );
    }

    // Short message
    cout << "LogCollector::enable: Logging enabled. Logfile is " << realFileName << endl;
    mLogTimer.restart();
    return true;
}


void LogCollector::disable() {
    if ( mEnabled ) {
        mOutStream->close();
        delete mOutStream;
        mOutStream = NULL;
    }
    mEnabled = false;
}


void LogCollector::addColumns(const std::list<string>& ids)
{
  for (std::list<string>::const_iterator it=ids.begin();it!=ids.end();++it) {
    addColumn(*it);
  }
}


bool LogCollector::addColumn(const string &id)
{
  addColumn(id,id,false);
}


bool LogCollector::addColumn( const string &id, const string &description, bool isTrigger ) {
    // Check if id already exists
    LogColumnMap::const_iterator idIter = mColMap.begin();
    for ( ; idIter != mColMap.end(); idIter++ ) {
        if ( idIter->first == id ) {
            // Id exists already
            return false;
        }
    }

    // Add new column
    LogColumn col;
    col.description = description;
    col.isTrigger = isTrigger;
    col.value = 0.0;
    col.colNum = mColNum;
    mColNum++;
    mColMap.insert( pair<string,int>( id, col.colNum ));
    mColData.push_back( col );

    // Write description if logging is enabled
    if ( mEnabled ) {
        writeDescription( col, mOutStream );
    }
}

bool LogCollector::addValue( const string &id, double value ) {
    if ( !mEnabled ) {
        return false;
    }

    LogColumnMap::iterator iter = mColMap.find( id );
    if ( iter == mColMap.end()) {
        // Id does not exist
        return false;
    }

    // Set value
    mColData[iter->second].value = value;

    // Check if the column triggers a new line in the logfile
    if ( mEnabled && mColData[iter->second].isTrigger ) {
        // Write new line to the logfile
        writeLine( mOutStream );
    }

    return true;
}

bool LogCollector::addValue( const string &id, int value ) {
    addValue( id, (double)value );
}

bool LogCollector::writeDescription( const LogColumn &col, ofstream * outStream ) {
    // Write column number, trigger flag and description
    *outStream << mCommentChar<< " Column " << col.colNum + 1 << ": " << col.description;
    *outStream << " (Trigger: " << ( col.isTrigger ? "yes)" : "no)" ) << endl;
    return true;
}


bool LogCollector::writeLogLine()
{
    return writeLine( mOutStream );
}


bool LogCollector::writeLine( ofstream * outStream ) {
    if ( !mEnabled ) {
        return false; // Not enabled, dont call this method
    }

    if ( mColNum <= 0 ) {
        return true; // No columns, nothing to do
    }

    // Write timestamp
 //   timeval now;
 //   gettimeofday( &now, 0 );
//    *outStream << scientific << Misc::getTimeDiff( &mEnableTime, &now ) << " ";
    *outStream << mLogTimer.msElapsed() << " ";
    // Write all colums
    ColumnVector::const_iterator colIter = mColData.begin();
    while ( colIter != mColData.end()) {
        *outStream << scientific << colIter->value << " ";
        colIter++;
    }
    *outStream << endl;

    return true;
}
