/**
  @file |FILE|
  
  (c) 2009 WSI-RA University of Tuebingen

  @author Karsten Bohlmann <bohlmann@gmail.com>

  @date Sun Apr 5 2009
*/
#ifndef GPSUBLOX_H
#define GPSUBLOX_H



#include "serialgps.h"
#include "ubloxmsg.h"

namespace GPS {

    const int UBLOX_BAUDRATE = 115200;

    /**
        driver class for UBlox 5/6 GPS modules
        using binary UBX protocol
        example usage:
        @code
            string serialDevice("/dev/ttyACM0");
            GPS::Ublox ublox(serialDevice);
            ublox.Connect();
            while (true) {
                ublox.Update();
                long longitude = ublox.getNAVPOSLLH().Lon_deg;
                long latitude = ublox.getNAVPOSLLH().Lat_deg;
                sleep(1);
            }
        @endcode
        @author karsten bohlmann <bohlmann@gmail.com>
        @author peter biber
        @author udo beck
    */
    class Ublox : public SerialGps
    {
    public:
    /**
        ctor
        @param serialDevice, e.g. "/dev/ttyACM0"
    */
        Ublox(const string& serialDevice);

        ~Ublox();

        /**
            reads one msg from GPS, parses data and updates class members
            read current data with get*-methods
            call this function regularly
            @return true if succesful, false if read error

        */
        virtual bool Update ();

        /**
            enables USB-output for the Ublox-Module and the
            the UBX Msgs NAV-SOL NAV-POSLLH NAV-VELNED NAV-DOP NAV-STATUS
        */
        virtual bool EnableUbxMsgs ();

        /**
            disables ubx msgs enabled by enableUbxMsgs()
        */
        virtual void DisableUbxMsgs ();

        /**
            resets the Ublox-Receiver Forced (watchdog) with hotstart
        */
        virtual void ResetUbx ();

        /**
            see ubx-documentation for data field decription
        */
        x01x06 getNAVSOL(){return NAVSOL;};
        x02x20 getRXMSVIS(){return RXMSVIS;};
        x02x10 getRSMRAW(){return RSMRAW;};
        x02x31 getRXMEPH(){return RXMEPH;};
        x01x12 getNAVVELNED(){return NAVVELNED;};
        x01x02 getNAVPOSLLH(){return NAVPOSLLH;};
        x01x04 getNAVDOP(){return NAVDOP;};
        x01x03 getNAVSTATUS(){return NAVSTATUS;};

    private:

        /**
         * Read data from the given serial device and print an error message if
         * no data is available
         */
        unsigned int readData( int serial_fd, char* c );

        /// True if the last package had a faulty checksum.
        bool cksum_err_;

        x02x20 RXMSVIS;
        x01x06 NAVSOL;
        x02x10 RSMRAW;
        x02x31 RXMEPH;
        x01x12 NAVVELNED;
        x01x04 NAVDOP;
        x0Bx02 AIDHUI;
        x01x02 NAVPOSLLH;
        x01x03 NAVSTATUS;

    };

}

#endif
