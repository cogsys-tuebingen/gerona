
///////////////////////////////////////////////////////////////////////////////
// I N C L U D E S
///////////////////////////////////////////////////////////////////////////////

// C/C++
#include <iostream>

// Project
#include "Strings.h"
#include "Avr32UiHandler.h"
#include "MsgPrint.h"

///////////////////////////////////////////////////////////////////////////////
// D E F I N E S
///////////////////////////////////////////////////////////////////////////////

// Filesize units
#define SIZEUNIT_BYTE 0
#define SIZEUNIT_KB   1
#define SIZEUNIT_MB   2
#define SIZEUNIT_GB   3

// Host data lines definitions. KEEP THEM CONSISTENT WITH THE AVR32 CODE
#define HOSTDATA_MAXLINES 20        // Maximum number of lines
#define HOSTDATA_LABELLENGTH 12     // Maximum length of label string
#define HOSTDATA_VALUELENGTH 18     // Maximum length of value string

///////////////////////////////////////////////////////////////////////////////
// I M P L E M E N T A T I O N
///////////////////////////////////////////////////////////////////////////////

using namespace std;

//// class Avr32UiHandler /////////////////////////////////////////////////////

Avr32UiHandler::Avr32UiHandler( RamaxxConnection *conn )
    : mConn( conn ), mLogStateReq( false ), mLogNumberReq( 0 ),
      mLogStateInfo( false ), mLogSizeInfo( 0 ), mLogSizeUnit( SIZEUNIT_BYTE ), mLogNumberInfo( 0 ),
      mMaxDataLines( HOSTDATA_MAXLINES ), mHostDataLineReq( false )
{
    U8 msgTypes[] = { MT_LOGCTRL, MT_HOST_DATA_LINE };
    mConn->addQuMsgHandler( msgTypes, sizeof( msgTypes), *this );
    mUpdateWatch.restart();
}

void Avr32UiHandler::getLogRequest( bool &state, int &number ) {
    state = mLogStateReq;
    number = mLogNumberReq;
}

void Avr32UiHandler::setLogInfo( bool state, U16 number, U32 filesize ) {
    bool imUpdate = ( mLogStateInfo != state );

    mLogStateInfo = state;
    mLogNumberInfo = number;

    // Scaled filesize and unit
    if ( filesize > 1E9 ) { // GB
        mLogSizeUnit = SIZEUNIT_GB;
        filesize /= 1E9;
    } else if ( filesize > 1E6 ) { // MB
        mLogSizeUnit = SIZEUNIT_MB;
        filesize /= 1E6;
    } else if ( filesize > 1E3 ) { // KB
        mLogSizeUnit = SIZEUNIT_KB;
        filesize /= 1E3;
    } else {
        mLogSizeUnit = SIZEUNIT_BYTE;
    }
    mLogSizeInfo = (U16)filesize;

    if ( imUpdate )
        sendLogInfo();
}

void Avr32UiHandler::update() {
    // Send data lines if requested
    if ( mHostDataLineReq ) {
        mHostDataLineReq = false;
        for ( map<int,Avr32HostDataLine>::iterator it=mDisplayMap.begin();it!=mDisplayMap.end();++it) {
            Avr32HostDataLine& line=it->second;
            if ( line.changed ) { // Data changed => transmit
                sendDataLine( line.index, line.label, line.value );
                line.changed = false;
            }
        }
    }

    if ( mUpdateWatch.sElapsed() < 0.75 )
        return; // Nothing to do

    // Reset update stopwatch
    mUpdateWatch.restart();

    // Send log info
    sendLogInfo();
}

bool Avr32UiHandler::setDataLine( int id, const std::string& value ) {
    map<int,Avr32HostDataLine >::iterator it=mDisplayMap.find(id);
    if (it==mDisplayMap.end()) {
        return false;
    }

    // Check value length and copy
    if ( value.size() > HOSTDATA_VALUELENGTH ) {
        it->second.value = value.substr( 0, HOSTDATA_VALUELENGTH );
    } else {
        it->second.value = value;
    }
    it->second.changed = true;

    return true;
}


void Avr32UiHandler::activateDataLine(int id, const std::string &label)
{
    map<int,Avr32HostDataLine >::iterator it=mDisplayMap.find(id);
    string lb;
    if (label.length()>HOSTDATA_LABELLENGTH) {
        lb = label.substr(0,HOSTDATA_LABELLENGTH);
    } else {
        lb = label;
    }
    if (it==mDisplayMap.end()) {
        if (mDisplayMap.size()>=mMaxDataLines) {
            ERRORPRINT( "Too many data lines in AVR32 Host Display" );
            return;
        }
        mDisplayMap[id]=Avr32HostDataLine(lb,"",mDisplayMap.size());

    } else {
        it->second.label=lb;
        it->second.value="";
    }
}

void Avr32UiHandler::processQuMessage( const QuMessage &msg ) {
    switch ( msg.type ) {
    case MT_LOGCTRL:
        processLogCtrlMsg( msg );
        break;
    case MT_HOST_DATA_LINE:
        mHostDataLineReq = true;
        break;
    }
}

void Avr32UiHandler::processLogCtrlMsg( const QuMessage &msg ) {
    if ( msg.length < 3 ) {
        ERRORPRINT( "Received log control message with invalid size. Skipping." );
        return;
    }

    mLogStateReq = ( msg.data[0] != 0 );
    mLogNumberReq = 0;
    mLogNumberReq |= msg.data[1] << 8;
    mLogNumberReq |= msg.data[2];
}

void Avr32UiHandler::sendLogInfo() {
    QuMessage msg;
    if ( mLogStateInfo ) {
        // Send enabled state etc
        msg.data.resize( 6 ); // State (1), number (2), filesize (2), filesize unit (1)
        msg.data[0] = 0x01;
        msg.data[1] = mLogNumberInfo >> 8;
        msg.data[2] = mLogNumberInfo & 0xFF;
        msg.data[3] = mLogSizeInfo >> 8;
        msg.data[4] = mLogSizeInfo & 0xFF;
        msg.data[5] = mLogSizeUnit;
    } else {
        // Send disabled state
        msg.data.resize( 1 );
        msg.data[0] = 0x00;
    }
    msg.length = msg.data.size();
    msg.type = MT_LOGSTATUS;
    mConn->queueMsg( msg );
}

void Avr32UiHandler::sendDataLine( U8 index, const string& label, const string& value ) {
    U16 msgLength = 3;

    msgLength += label.size();
    msgLength += value.size();

    if ( msgLength > MAX_MSG_LENGTH )
        return;

    QuMessage msg;
    msg.data.resize( msgLength );
    msg.length = msgLength;
    msg.data[0] = index;
    msg.data[1] = label.size();
    msg.data[2] = value.size();

    // TODO Cleanup
   const char * cStr = label.c_str();
   for ( int i = 0; i < label.size(); ++i ) {
       msg.data[i + 3] = cStr[i];
   }
   cStr = value.c_str();
   for ( int i = 0; i < value.size(); ++i ) {
       msg.data[i + 3 + label.size()] = cStr[i];
   }
   msg.type = MT_HOST_DATA_LINE;
   mConn->queueMsg( msg );
}


