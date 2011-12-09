/**
  @file |FILE|
  
  (c) 2009 WSI-RA University of Tuebingen

  @author Karsten Bohlmann
  @email bohlmann@gmail.com

  @date Sun Apr 5 2009
*/

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include "serialgps.h"
#include "types.h"

namespace GPS {

    SerialGps::SerialGps(const string& serialDevice, int baudRate)
        : mSerialDevice(serialDevice),mBaudRate(baudRate)
    {
        mSerialOpen = false;
        mSerialFd=-1;
    }


    SerialGps::~SerialGps()
    {
    }


    int SerialGps::Connect()
    {
        if (mSerialOpen) {
            return EOK;
        }
        int status=openSerial(&mSerialFd,mSerialDevice.c_str(),mBaudRate);
        if (status==EOK) {
            mSerialOpen=true;
            return EOK;
        } else {
            mSerialOpen=false;
            return status;
        }

    }



    int SerialGps::Disconnect ()
    {
        if (mSerialOpen) {
            close(mSerialFd);
            mSerialOpen = false;
        }
        return EOK;
    }


    int SerialGps::openSerial(int *fdptr, const char *serialDevice, int baudRate)
    {
        int baudFlag;
        switch (baudRate) {
        case 9600:
            baudFlag = B9600;
            break;
        case 19200:
            baudFlag = B19200;
            break;
        case 57600:
            baudFlag = B57600;
            break;
        case 115200:
            baudFlag = B115200;
            break;
        default:
            baudFlag = B57600;
            break;
        }

        cout << "opening " << serialDevice << endl;
        
        *fdptr = open(serialDevice, O_RDWR | O_NOCTTY );
        if (*fdptr <0) {
            cout << "error opening" << serialDevice <<endl;
            return EHW;
        }
        
        tcgetattr(*fdptr,&mOldtio); /* save current port settings */
        
        memset(&mNewtio,0, sizeof(mNewtio));
        //newtio.c_cflag = baudRate | CRTSCTS | CS8 | CLOCAL | CREAD;
        mNewtio.c_cflag = baudFlag |  CS8 | CLOCAL | CREAD;
        mNewtio.c_iflag = IGNPAR;
        mNewtio.c_oflag = 0;
        
        /* set input mode (non-canonical, no echo,...) */
        mNewtio.c_lflag = 0;

        mNewtio.c_cc[VTIME]    = 0;   /* inter-character timer unused */
        mNewtio.c_cc[VMIN]     = 1;   /* blocking read until 5 chars received */
        
        tcflush(*fdptr, TCIFLUSH);
        tcsetattr(*fdptr,TCSANOW,&mNewtio);
        cfsetspeed(&mNewtio,baudFlag);
        
        return EOK;
    }


}
