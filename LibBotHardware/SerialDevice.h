#ifndef __SERIAL_DEVICE_H__
#define __SERIAL_DEVICE_H__

enum SERIAL_RESULT_TYPE {
	SERIAL_OK = 0,                    // Everything's OK.
	SERIAL_PORT_ALREADY_OPENED = 1,   // Can't re-open an already-opened serial port.
	SERIAL_CANNOT_OPEN_CONNECTION = 2 // Can´t open the serial port.
};

/** @class SERIAL
  *
  * @brief This class provides the interface to a serial port.
  *
  * It is a wrapper class for the carmen serial port functionality.
  **/
class SerialDevice
{
  public:
    SerialDevice(void) : dev_fd(0), connected(false) {}
    ~SerialDevice(void) {}

    SERIAL_RESULT_TYPE connect (const char* comPort);
    bool close();
    void send(unsigned char *data, unsigned length);
    void receive(unsigned char *data, unsigned length);

    bool isConnected();

	private:
		int dev_fd;
    bool connected;
};

#endif
