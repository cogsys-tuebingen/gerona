/*
 * types.h
 *
 *  Created on: 19.08.2009
 *      Author: radbarbeit5
 */

#ifndef QUMESSAGES_H_
#define QUMESSAGES_H_

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

#include "Global.h"
#include <list>

///////////////////////////////////////////////////////////////////////////////
// DEFINITIONS
///////////////////////////////////////////////////////////////////////////////

// Message id

// KEEP THIS DEFINITIONS CONSISTENT WITH THE CORRESPONDING DEFINITIONS IN
// Avr32/RobotControl/Includes/quMessage.h !

// Reserved IDs from QuBast, do not change them
#define MT_STRING               51
#define MT_FLIGHTDATA           52
#define MT_SONAR                53
#define MT_WIICAM               54
#define MT_ADNS_MOVES           55
#define MT_ADNS_DUMPS           56
#define MT_IR                   57
#define MT_CHANNEL_CONTROL      70
#define MT_WIICAM_CONFIG        71
#define MT_CONFIG               72
#define MT_MARKER               112

// Define new IDs here
#define MT_INVALID              19
#define MT_STEER1               20
#define MT_STEER2               21
#define MT_SPEED                22
#define MT_PAN                  23
#define MT_TILT                 24
#define MT_ADNS_ERROR           25
#define MT_ADNS_MODE            26
#define MT_ADNS_STATUS          27
#define MT_ADNS_PIXDUMP         28
#define MT_ADNS_POSITION        29
#define MT_BEEP                 30
#define MT_COMPASS              31
#define MT_VOLTAGE              102
#define MT_STRING_MAX_LEN       21
#define MT_FLODO_DIST           32
#define MT_WHEELENC             33
#define MT_PCFVT                34
#define MT_ADSVT                35
#define MT_EVX                  36
#define MT_LOGCTRL              37
#define MT_LOGSTATUS            38
#define MT_ACTUATOR             39
#define MT_HOST_DATA_LINE       40
#define MT_PARAM                41
#define MT_SPEEDCTRL            42
#define MT_FLASH_SAVE_DATA      253
#define MT_FLASH_LOAD_DATA      254
#define MT_FLASH_LOAD_DEFAULT   255


// Endian swap macros

// 16bit value
#define SWAP16(value) ( (((value)>>8) & 0x00ff) | (((value)<<8) & 0xff00) )

// 32bit value
#define SWAP32(value) ((((value)<<24) & 0xff000000) | \
(((value)<<8) & 0x00ff0000)| \
(((value)>>8) & 0x0000ff00)| \
(((value)>>24) & 0x000000ff))

// 64bit value (double or long long)
#ifdef WIN32
// visual c++ 6.0 does not know and need the ULL suffix
#define SWAP64(value)     ((((value)<<56) & 0xff00000000000000) | \
(((value)<<40) & 0x00ff000000000000)  | \
(((value)<<24) & 0x0000ff0000000000)  | \
(((value)<< 8) & 0x000000ff00000000)  | \
(((value)>> 8) & 0x00000000ff000000)  | \
(((value)>>24) & 0x0000000000ff0000)  | \
(((value)>>40) & 0x000000000000ff00)  | \
(((value)>>56) & 0x00000000000000ff))
#else
// ull = unsigned long long
#define SWAP64(value)     ((((value)<<56) & 0xff00000000000000ull) | \
(((value)<<40) & 0x00ff000000000000ull)  | \
(((value)<<24) & 0x0000ff0000000000ull)  | \
(((value)<< 8) & 0x000000ff00000000ull)  | \
(((value)>> 8) & 0x00000000ff000000ull)  | \
(((value)>>24) & 0x0000000000ff0000ull)  | \
(((value)>>40) & 0x000000000000ff00ull)  | \
(((value)>>56) & 0x00000000000000ffull))
#endif

///////////////////////////////////////////////////////////////////////////////
// DECLARATIONS
///////////////////////////////////////////////////////////////////////////////

const int BEEP_MSG_YES[]  = {60,60,0};
const int BEEP_MSG_FOUND[]  = {20,100,0};

/**
 *  this struct represents a binary message
 *  in qubast-format used for communication with a microcontroller
 *
 */
struct QuMessage
{
    /**
     * constructor
     */
    QuMessage () {type=MT_INVALID; length = 0;}

    /**
     * copy constructor
     *
     */
    QuMessage( const QuMessage& src )
    : type(src.type), length(src.length),data(src.data.begin(),src.data.end()) {
    }

    /**
     * constructs a qumessage object with given data+type
     */
    QuMessage (U8 type_, U16 length_, const U8 *data_)
    : type(type_),length(length_),data(data_,data_+length) {
        // v.resize ( n );
        // std::copy ( a, a + n, v.begin() );
    }
    U8  type;
    U16 length;
    vector<U8>  data;
};

typedef list<QuMessage> QuMessageList;

class QuMsgHandler {
public:

    /**
     * Destructor.
     */
    virtual ~QuMsgHandler() {}

    /**
     * Process the given message.
     *
     * @param msg The message.
     */
    virtual void processQuMessage( const QuMessage &msg ) { /* Empty */ };

    /**
     * Called if the connection to the microcontroller is
     * ready to use.
     */
    virtual void connectionEstablished() { /* Empty */ }

    /**
     * Called if the connection to the microcontroller was shutdown.
     */
    virtual void connectionClosed() { /* Empty */ }
};

/*
 * Message Data definitions follow here
 * Make sure these are equal to defs in AVR32/RobotControll/types.
 */

/**
 * compass data MT_COMPASS
 */
typedef struct TTypeSpecificCMP     TypeSpecificCMP;
struct TTypeSpecificCMP {

    static const int ERROR_STATE = ( 1 << 4 );

    U8 state;
    U16 heading;
};


#endif /* QUMESSAGES_H_ */
