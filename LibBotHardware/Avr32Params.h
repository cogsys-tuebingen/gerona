#ifndef AVR32PARAMS_H
#define AVR32PARAMS_H


///////////////////////////////////////////////////////////////////////////////
// I N C L U D E S
///////////////////////////////////////////////////////////////////////////////

// Project
#include <Global.h>
#include "QuMessage.h"
#include "UsbConn.h"
#include "Actuators.h"
#include "RobotSensors.h"

///////////////////////////////////////////////////////////////////////////////
// D E F I N I T I O N S
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// D E C L A R A T I O N S
///////////////////////////////////////////////////////////////////////////////

class Avr32Params : public QuMsgHandler {
public:
   /**
    * Constructor.
    *
    * @param conn Connection to the robot.
    * @param sensors The robots sensors.
    * @para actuators The robots actuators.
    */
   Avr32Params( RamaxxConnection * conn, RobotSensors * sensors, Actuators *actuators );

   /* Inherited from QuMsgHandler */
   void processQuMessage( const QuMessage &msg ) { /* Nothing to do */ }

   /* Inherited from QuMsgHandler */
   void connectionEstablished();

private:


   /**
    * Transmit all parameters to the Avr32.
    */
   void sendAllParams();

   /**
    * Transmitt one parameter to the Avr32.
    *
    * @param type Parameter type.
    * @param subtype Parameter subtype.
    * @param val Parameter value.
    */
   void sendParam( const U8& type, const U8& subtype, const S32& val );

   /// Connection to the robot
   RamaxxConnection * mConn;

   /// The robots sensors
   RobotSensors * mSensors;

   /// The robots actuators
   Actuators * mActuators;

   /// True if we transmitted the parameters already
   bool mParamsSend;
};

#endif // AVR32PARAMS_H
