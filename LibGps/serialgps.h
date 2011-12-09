/**
  @file |FILE|
  
  (c) 2009 WSI-RA University of Tuebingen

  @author Karsten Bohlmann
  @email bohlmann@gmail.com

  @date Sun Apr 5 2009
*/
#ifndef GPSSERIALGPS_H
#define GPSSERIALGPS_H
#include <termios.h>
#include "types.h"
namespace GPS {

/**
	@author karsten bohlmann <bohlmann@gmail.com>
    base class for GPS-devices connected by serial port
    (real port or usb-emulation)
*/
class SerialGps
{
public:
    /**
      ctor
      @param serialDevice filename of serialdevice,e.g. "/dev/ttyACM0"
      */
  SerialGps(const string& serialDevice, int baudrate);
  
  ~SerialGps();

  /**
    call this to connect to GPS device
    */
  int Connect ();
  virtual bool Update() = 0;

  /**
    disconnect from GPS device
    */
  int Disconnect ();

protected:
  string mSerialDevice;
  int openSerial(int *fdptr, const char *serialDevice, int baudRate); 
  int mSerialFd;
  bool mSerialOpen;
  int mBaudRate;
  ostream *mLog;
  struct termios mOldtio,mNewtio;

};

}

#endif
