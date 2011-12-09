/*
 * RamaxxUsb.cpp
 *
 *  Created on: Jan 21, 2010
 *      Author: radbarbeit10
 */

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

// C/C++
#include <iostream>
#include <cstring>
#include <time.h>

// Workspace
#include <Misc.h>

// Project
#include "UsbConn.h"
#include "Ramaxx.h"
#include "MsgPrint.h"

///////////////////////////////////////////////////////////////////////////////
// IMPLEMENTATION
///////////////////////////////////////////////////////////////////////////////

using namespace std;

//// class RamaxxConnection ///////////////////////////////////////////////////

RamaxxConnection::RamaxxConnection() {
    usb_init();
	mDevHandle = NULL;
        msgInCnt=msgOutCnt = 0;
        for (int i=0;i<256;++i) {
            mMsgStatsMap[i]=0;
        }
}

RamaxxConnection::~RamaxxConnection() {
	closeAvr32();
}

struct usb_device * RamaxxConnection::searchAvr32() {
    // Find busses & devices
    usb_find_busses();
    usb_find_devices();

    // Search AVR32
    struct usb_bus *bus;
    bus = usb_get_busses();
    for ( ; bus; bus = bus->next ) {
        struct usb_device *dev;
        for ( dev = bus->devices; dev; dev = dev->next ) {
            struct usb_device_descriptor desc;
            desc = dev->descriptor;
            if ( desc.idProduct == USB_iPRODUCT && desc.idVendor == USB_iVENDOR ) {
                return dev;
            }
        }
    }
    return NULL;
}

bool RamaxxConnection::isAvr32Present() {
    return searchAvr32() != NULL;
}

bool RamaxxConnection::openAvr32() {
    // Clear msg queue
    while ( !mMsgQueue.empty())
        mMsgQueue.pop();

    // Close Avr32?
    if ( isOpen()) {
        closeAvr32();
    }

	// Search device
    struct usb_device * dev = searchAvr32();
	if ( dev == NULL )
		return false;

	// Open device
	struct usb_dev_handle * handle = usb_open( dev );
	if ( handle == NULL )
	    return false;

	// Claim interface
	if ( usb_claim_interface( handle, 0 ) < 0 )
	    return false;

	mDevHandle = handle;
	mErrorCount = 0;
	mHasMarker = true;
  msgCntTimer.restart();
  fireConnectionEstablished();

	return true;
}

bool RamaxxConnection::isOpen() {
	return mDevHandle != NULL;
}

void RamaxxConnection::closeAvr32() {
	if ( isOpen()) {
		usb_close( mDevHandle );
    fireConnectionClosed();
	}
	mDevHandle = NULL;
}

bool RamaxxConnection::queueMsg( const QuMessage &msg ) {
    if ( isOpen() && msg.length <= ( MAX_MSG_LENGTH )) {
        mMsgQueue.push( msg );
        return true;
    }
    return false;
}

bool RamaxxConnection::sendMsgs() {
    bool error = false;

    // USB send buffer
    U8 buffer[MAX_MSG_LENGTH + MSG_HEADER_LENGTH];

    // Send all messages
    while ( !mMsgQueue.empty()) {
        // Copy message into send buffer
        QuMessage msg = mMsgQueue.front();
        buffer[0] = msg.type;
        buffer[1] = ( msg.length >> 8 ) & 0xFF;
        buffer[2] = msg.length & 0xFF;
        for ( size_t i = 0; i < msg.length; ++i ) {
            buffer[i + 3] = msg.data[i];
        }
        // Cannot send messages if the device is not open
        if ( !isOpen()) {
            return false;
        }
        // Send message data
        if ( !sendData( buffer, msg.length + MSG_HEADER_LENGTH )) {
            error = true;
            break;
        }
        // Message was send, remove it from the queue
        U8 msgType = msg.type;
        mMsgQueue.pop();
        // ***hack debug code
        ++msgOutCnt;
        int typus= msg.type;
        int count=mMsgStatsMap[typus];
        mMsgStatsMap[typus]=count+1;

        // ***end hack
        // If this was the communication token, stop sending
        if ( msgType == MT_MARKER ) {
            mHasMarker = false;
            break;
        }
    }
    // Increment error counter if necessary
    if ( error )
        mErrorCount++;

    return !error;
}

bool RamaxxConnection::processRobotMsgs() {
    U8 buffer[64];
    int msgDataPos = 0;
    int size = 0;
    int state = 0;
    int rxByteCount = 0;

    // Cannot read messages if the device is not open
    if ( !isOpen()) {
        return false;
    }

    // Send all queued messages and the token if we have the token
    if ( mHasMarker ) {
        QuMessage marker( MT_MARKER, 0, NULL );
        queueMsg( marker );
        mHasMarker = false;
        if ( !sendMsgs()) {
            //player: ERRORPRINT( "ERROR. Cannot send messages" );
            mErrorCount++;
            return false;
        }
    }
    if (msgCntTimer.msElapsed()>1000) {
      //  cout << "Outmsgcnt:"<<msgOutCnt << " msgincnt:"<<msgInCnt << " errors:"<<mErrorCount<<endl;

        /*for (std::map<int,int>::iterator it=mMsgStatsMap.begin();it!=mMsgStatsMap.end();++it) {
            if (it->second!=0) {

                cout << "type:"<<it->first << " count::"<<it->second << " ";
                it->second=0;
            }
        }
        cout << endl;
        msgOutCnt=msgInCnt = mErrorCount = 0;
        msgCntTimer.restart();*/
    }
    timeval start;
    gettimeofday( &start, 0 );
    // Try to read until the token is received
    QuMessage msg;
    while ( !mHasMarker ) {
        // Read one packet
        if ( !readData( buffer, &size )) {
            mErrorCount++;
            ERRORPRINT( "ERROR. Cannot read data from the AVR32" );
            return false;
        }
        rxByteCount += size;
        // Process the packet data
        for ( int i = 0; i < size && !mHasMarker; ++i ) {
            switch ( state ) {
            // Read type
            case 0:
                msg.type = buffer[i];
                state = 1;
                msgDataPos = 0;
                break;

            // Read length byte 1
            case 1:
                msg.length = 0;
                msg.length |= ( buffer[i] << 8 );
                state = 2;
                break;

            // Read length byte 0, relay message if message length is 0
            case 2:
                msg.length |= buffer[i];
                if ( msg.length > 1024 ) {
                    ERRORPRINT( "ERROR. Invalid message length." );
                    ERRORPRINT1( "Message length was: %d", msg.length );
                    mErrorCount++;
                    state = 0;
                    break;
                }
                if ( msg.length > 0 ) {
                    msg.data.resize( msg.length );
                    state = 3;
                } else {
                    relayQuMessage( msg );
                    ++msgInCnt;
                    if ( msg.type == MT_MARKER ) {
                        mHasMarker = true;
                    }
                    state = 0;
                }
                break;

            // Read message data
            case 3:
                msg.data[msgDataPos++] = buffer[i];
                if ( msg.length <= msgDataPos ) {
                    relayQuMessage( msg );
                    ++msgInCnt;
                    if ( msg.type == MT_MARKER ) {
                        mHasMarker = true;
                    }
                    state = 0;
                }
            }
        }
    }

    // Check if the last message was complete
    if ( state != 0 ) {
        mErrorCount++;
         ERRORPRINT( "ERROR. Received an invalid message" );
    }

    timeval end;
    gettimeofday( &end, 0 );

//    cout << "Received " << rxByteCount << " bytes in " << Misc::getTimeDiff( &start, &end );
//    cout << " seconds." << endl;
    return true;
}

bool RamaxxConnection::readData( U8 * buffer, int * size ) {
    if ( !isOpen()) {
        return false;
    }

    // Try to read
    *size = usb_bulk_read( mDevHandle, USB_EP_IN, (char*)buffer, 64, 100 );
    if ( *size < 0 ) {
        ERRORPRINT1( "ERROR. Cannot read data from Avr32. Return value was: %d", *size );
        return false;
    }
    return true;
}

bool RamaxxConnection::sendData( U8 * data, int size ) {
	if ( !isOpen()) {
		return false;
	}
	int ret = usb_bulk_write( mDevHandle, USB_EP_OUT, (char*)data, size, 10 );
	if ( ret != size ) {
        ERRORPRINT1( "ERROR. Cannot send data to Avr32. Return value was: %d", ret );
	    return false;
	}
	return true;
}

bool RamaxxConnection::sendStdRequest( int request ) {
	if (!isOpen()) {
		return false;
	}
	return 0 <= usb_control_msg(
			mDevHandle,
			USB_RECIP_DEVICE | USB_TYPE_VENDOR | USB_ENDPOINT_IN,
			request,
			0x00,
			0x00,
			NULL,
			0,
			100 );
}

bool RamaxxConnection::sendSetupRequest() {
    return sendStdRequest( USB_SETUP_RQ );
}

bool RamaxxConnection::sendWdtResetRequest() {
    return sendStdRequest( USB_WDT_REST_RQ );
}

bool RamaxxConnection::sendShutdownRequest() {
    return sendStdRequest( USB_SHUTDOWN_RQ );
}

void RamaxxConnection::addQuMsgHandler( U8 msgType, QuMsgHandler &handler ) {
    QuMsgListenerMap::iterator iter = mListenerMap.find( msgType );
    if ( iter != mListenerMap.end()) {
        iter->second->push_back( &handler );
    } else {
        vector<QuMsgHandler*>* vec = new vector<QuMsgHandler*>();
        vec->push_back( &handler );
        mListenerMap.insert( make_pair( msgType, vec ));
    }
}

void RamaxxConnection::addQuMsgHandler( U8 msgTypes[], size_t length, QuMsgHandler &handler ) {
    for ( size_t i = 0; i < length; ++i ) {
        addQuMsgHandler( msgTypes[i], handler );
    }
}

void RamaxxConnection::addQuMsgHandler( QuMsgHandler &handler ) {
    mListener.push_back( &handler );
}

void RamaxxConnection::relayQuMessage( const QuMessage &msg ) {
    //cout << (int)msg.type << endl;
    QuMsgListenerMap::iterator iter = mListenerMap.find( msg.type );
    if ( iter != mListenerMap.end()) {
        vector<QuMsgHandler*> lis = *iter->second;
        for ( size_t i = 0; i < lis.size(); ++i ) {
            lis[i]->processQuMessage( msg );
        }
    }
    for ( size_t i = 0; i < mListener.size(); ++i ) {
        mListener[i]->processQuMessage( msg );
    }
}

void RamaxxConnection::fireConnectionClosed() {
    for ( size_t i = 0; i < mListener.size(); ++i )
        mListener[i]->connectionClosed();
    QuMsgListenerMap::iterator iter = mListenerMap.begin();
    for ( ; iter != mListenerMap.end(); iter++ ) {
        vector<QuMsgHandler*> lis = *iter->second;
        for ( size_t i = 0; i < lis.size(); ++i ) {
            lis[i]->connectionClosed();
        }
    }
}

void RamaxxConnection::fireConnectionEstablished() {
    for ( size_t i = 0; i < mListener.size(); ++i )
        mListener[i]->connectionEstablished();
    QuMsgListenerMap::iterator iter = mListenerMap.begin();
    for ( ; iter != mListenerMap.end(); iter++ ) {
        vector<QuMsgHandler*> lis = *iter->second;
        for ( size_t i = 0; i < lis.size(); ++i ) {
            lis[i]->connectionEstablished();
        }
    }
}
