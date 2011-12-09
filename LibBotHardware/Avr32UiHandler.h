#ifndef AVR32UIHANDLER_H
#define AVR32UIHANDLER_H


///////////////////////////////////////////////////////////////////////////////
// I N C L U D E S
///////////////////////////////////////////////////////////////////////////////

// C/C++
#include <string>
#include <vector>
#include <map>

// Workspace
#include <Stopwatch.h>
#include <Global.h>

// Project
#include "QuMessage.h"
#include "UsbConn.h"
using namespace std;

///////////////////////////////////////////////////////////////////////////////
// D E F I N I T I O N S
///////////////////////////////////////////////////////////////////////////////

enum {
    AVR_DISPLAY_PNI,
    AVR_DISPLAY_LASER_FRONT,
    AVR_DISPLAY_LASER_REAR,
    AVR_DISPLAY_GPS_EASTING,
    AVR_DISPLAY_GPS_NORTHING,
    AVR_DISPLAY_GPS_EXTRA,
    AVR_DISPLAY_SLAM_XY,
    AVR_DISPLAY_SLAM_COURSE
};



///////////////////////////////////////////////////////////////////////////////
// D E C L A R A T I O N S
///////////////////////////////////////////////////////////////////////////////

class Avr32HostDataLine {
public:

    Avr32HostDataLine() {
        label = "";
        value = "";
        changed = false;
    }
    Avr32HostDataLine(const string& l, const string& val,U8 idx)
        :label(l),value(val),index(idx),changed(false){}
    Avr32HostDataLine(const Avr32HostDataLine& src)
        :label(src.label),value(src.value),index(src.index),changed(src.changed) {}

    std::string label;
    std::string value;
    U8   index;
    bool changed;
};


/**
 * This class handles all user interface messages received from the Avr32
 * microcontroller and sends the necessary information back.
 */
class Avr32UiHandler : public QuMsgHandler {
public:

    /**
     * Constructor.
     *
     * @param conn Connection to the robot.
     */
    Avr32UiHandler( RamaxxConnection * conn );

    /**
     * Returns the latest requested logging state and logfile number.
     *
     * @param requestedState Latest requestet logging state (on/off)
     * @param requestedNumber Latest requested logging number
     */
    void getLogRequest( bool &requestedState, int &requestedNumber );

    /**
     * Set the logging information. This information will be send to the Avr32
     * microcontroller.
     *
     * @param state Current logging state (on/off).
     * @param number Current logfile number (set to zero if logging is disabled)
     * @param filesize Current logfile size [MB].
     */
    void setLogInfo( bool state, U16 number, U32 filesize );

    /**


     */
    void activateDataLine (int id, const std::string& label);

    /**
     * Send a status value to the Avr32.
     *
     * @param index Line number from 0 to 20
     * @param label Label string
     * @param value Value string
     */
    bool setDataLine( int id, const std::string& value );

    /**
     * Sends the necessary information to the Avr32.
     */
    void update();

    /* Inherited from QuMsgHandler */
    virtual void processQuMessage( const QuMessage &msg );

private:
    /**
     * Send logging information to the Avr32.
     */
    void sendLogInfo();

    /**
     * Send a data msg to Avr32 (displayed in screen host messages).
     *
     * @param index Line index
     * @param label Label string
     * @param value Value string
     */
    void sendDataLine( U8 index, const string& label, const string& value );

    /**
     * Process a log control message from the Avr32.
     */
    void processLogCtrlMsg( const QuMessage &msg );

    /** Connection to the robot */
    RamaxxConnection *mConn;
    /** Latest logging state request */
    bool mLogStateReq;
    /** Latest logfile number request */
    U16  mLogNumberReq;
    /** Logging state info */
    bool mLogStateInfo;
    /** Logfile size info */
    U16  mLogSizeInfo;
    /** Unit of logfile size */
    U8   mLogSizeUnit;
    /** Logfile number info */
    U16  mLogNumberInfo;
    /** Update stopwatch */
    Stopwatch mUpdateWatch;
    /** Status msg data */

    /** Host status data requested? */
    bool mHostDataLineReq;

    map<int,Avr32HostDataLine > mDisplayMap;
    int mMaxDataLines;
};

#endif // AVR32UIHANDLER_H
