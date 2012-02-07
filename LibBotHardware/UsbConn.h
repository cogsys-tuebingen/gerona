/*
 * RamaxxUsb.h
 *
 *  Created on: Jan 21, 2010
 *      Author: marks
 */

#ifndef USBCONN_H_
#define USBCONN_H_

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

// C/C++
#include <queue>
#include <map>

// Libusb
#include <usb.h>

// Project
#include "QuMessage.h"
#include "Stopwatch.h"
///////////////////////////////////////////////////////////////////////////////
// DEFINITIONS
///////////////////////////////////////////////////////////////////////////////

// Maximum message length in bytes (without the three header bytes)
#define MAX_MSG_LENGTH      61
// Length of the header in bytes
#define MSG_HEADER_LENGTH   3

// USB endpoint numbers
#define USB_EP_CONTROL      0x00
#define USB_EP_OUT          0x01
#define USB_EP_IN           0x82

// USB Pid & Vid
#define USB_iPRODUCT 	    0x0F0F
#define USB_iVENDOR		    0x03EB

// Request numbers
#define USB_SETUP_RQ        0x10
#define USB_WDT_REST_RQ     0x11
#define USB_SHUTDOWN_RQ     0x12

// Message vector
typedef queue<QuMessage> QuMsgQueue;

// Message listeners map
typedef map<U8, vector<QuMsgHandler*>* > QuMsgListenerMap ;

///////////////////////////////////////////////////////////////////////////////
// DECLARATIONS
///////////////////////////////////////////////////////////////////////////////

class RamaxxConnection {
public:

    /**
     * Contructor. Does not open the device.
     */
	RamaxxConnection();

	/**
	 * Destructor. Closes the device.
	 */
	virtual ~RamaxxConnection();

	/**
	 * Checks if the AVR32 is present.
	 *
	 * @return
	 *     True if the device is available.
	 */
	bool isAvr32Present();

	/**
	 * Opens the device.
	 *
	 * @return
	 *     False if there was any error, true otherwise.
	 */
	bool openAvr32();

	/**
	 * Checks if the device is opened.
	 *
	 * @return
	 *     True if the device is open and ready.
	 */
	bool isOpen();

	/**
	 * Closes the AVR32 device. Any errors will be ignored.
	 */
	void closeAvr32();

	/**
	 * Sends a setup request to the AVR32.
	 *
	 * @return
	 *     False if any error occurred, true otherwise.
	 */
	bool sendSetupRequest();

	/**
	 * Sends a watchdog reset request to the AVR32.
	 *
	 * @return
     *     False if any error occurred, true otherwise.
	 */
	bool sendWdtResetRequest();

	/**
     * Sends a shutdown reset request to the AVR32.
     *
     * @return
     *     False if any error occurred, true otherwise.
     */
	bool sendShutdownRequest();

	/**
	 * Queues a message. Invoke sendMsgs the send all queued messages.
	 *
	 * @param msg
	 *     The message.
	 *
	 * @return
	 *     True if the message length is valid and the message was
	 *     pushed on the queue, false otherwise.
	 */
	bool queueMsg( const QuMessage &msg );

	/**
	 * Sends all queued messages.
	 *
	 * @return
	 *     True if all messages were send without an error, false otherwise.
	 */
	bool sendMsgs();

	/**
	 * Reads all robot messages and relays them to the message listeners.
	 *
	 * @return
	 *     True if there was no error, false otherwise.
	 */
	bool processRobotMsgs();

	/**
     * Register a message handler. The handler will receive messages from the robot.
     *
     * @param msgType Type of the message that will be relayed to the handler.
     * @param handler The message handler.
     */
    void addQuMsgHandler( U8 msgType, QuMsgHandler &handler );

    /**
     * Register a message handler. The handler will receive messages from the robot.
     *
     * @param msgType Types of the messages that will be relayed to the handler.
     * @param length Size of msgTypes.
     * @param handler The message handler.
     */
    void addQuMsgHandler( U8 msgTypes[], size_t length, QuMsgHandler &handler );

    /**
     * Register a message handler. The handler will receive all messages.
     *
     * @param handler The message handler.
     */
    void addQuMsgHandler( QuMsgHandler &handler );

private:
	bool sendStdRequest( int request );
	bool sendData( unsigned char * data, int size );
	bool readData( unsigned char * buffer, int * size );
	bool readMsg( U8 * buffer, int bufferSize, int * bufferPos, QuMessage &msg );
	struct usb_device * searchAvr32();

	/**
     * Relays a message to all registered listeners.
     *
     * @param msg The message.
     */
    void relayQuMessage( const QuMessage &msg );

    void fireConnectionEstablished();
    void fireConnectionClosed();

	/** The USB device, NULL if not opened. */
	usb_dev_handle * mDevHandle;

	/** Message queue. */
	QuMsgQueue mMsgQueue;
	/** Error counter. */
	U32        mErrorCount;

	/** Message handlers listening to specific message types. */
    QuMsgListenerMap        mListenerMap;
    /** Message handlers listening to all message types. */
    vector<QuMsgHandler*>   mListener;

    /** True if we have the communication marker. */
    bool mHasMarker;
    int msgOutCnt,msgInCnt;
    std::map<int,int> mMsgStatsMap;
    Stopwatch msgCntTimer;
};

#endif /* USBCONN_H_ */
