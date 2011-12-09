#include "SerialDevice.h"
#include "CarmenSerial.h"

#include <iostream>
#include <string.h>

SERIAL_RESULT_TYPE SerialDevice::connect(const char* comPort)
{
  if (connected == true) {
    return SERIAL_PORT_ALREADY_OPENED;
  }
  char localPort[256];
    strncpy(localPort,comPort,254);
    if (carmen_serial_connect(&dev_fd, localPort) == -1) {
		return SERIAL_CANNOT_OPEN_CONNECTION;
	}

	carmen_serial_configure(dev_fd, 0, "N");
	connected = true;
	return SERIAL_OK;
}


bool SerialDevice::close()
{
	if (connected) {
		carmen_serial_close(dev_fd);
    return true;
  }
  return false;
}


void SerialDevice::send( unsigned char *data, unsigned length )
{
	carmen_serial_writen(dev_fd, data, length);
}


void SerialDevice::receive( unsigned char *data, unsigned length )
{
	carmen_serial_readn(dev_fd, data, length);
}

bool SerialDevice::isConnected() {
 return connected;
}
