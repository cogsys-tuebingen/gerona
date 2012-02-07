#ifndef __LASER_FX6_H__
#define __LASER_FX6_H__

#include "Basetime.h"
#include "SerialDevice.h"
#include <string>
#include <vector>
using namespace std;


/** @class LaserFx6
    *
    * @brief This class provides functions for scanning and logging with the fx6 laserscanner.
    */
class LaserFx6
{
  public:
        /// Constructs an instance of LaserFx6.
        /** @param Tx Rotation cycle of inner axle, Eigen value of each sensor (is displayed under the sensor back)
            * @param Ty Rotation cycle of outer axle, Eigen value of each sensor (is displayed under the sensor back)
            * @param pixNumHor Pixel numbers horizontally
            * @param pixNumVert Pixel numbers vertically
            */
    LaserFx6(unsigned tx, unsigned ty, unsigned resx, unsigned resy);
    /// Destroys the instance of LaserFx6.
        /** The scanning process is stopped, if running. */
        ~LaserFx6();

        /// Establishes a connection through the serial port comPort.
        bool connect(const char* comPort);
    /// Closes the connection through the serial port.
        bool close();
        /// Starts the scanning process.
    bool start();
        /// Stops the scanning process.
    bool stop();

        /// Reads a block of data.
        /** The data block contains either a response command, XY data or ranging data.
            */
    char readData();

        /// Sends the command to change the frame rate to fps.
    bool setFPS(unsigned char fps);
        /// Returns the current command.
        unsigned char getCurrentCmd() {return currentCmd;}
        /// Returns the current frame (that is 0 or 1).
        unsigned getCurrentFrame() {return currentFrame;}
        /// Returns the response code detail of the last command response.
        unsigned char getResponseCodeDetails() {return responseCodeDetails;}
        /// Returns the response data details of the last command response.
        unsigned char getResponseDataDetails() {return responseDataDetails;}
        /// Returns true, if the last CRC test is passed, false otherwise.
        bool isCRCPassed() {return CRC;}

        /// Returns the sorted light volume data.
        void getLightVolume(vector<float>&);
        /// Returns the sorted distance data.
        void getDistance(float*);
        /// Returns the sorted position data.
        void getPosition(vector<float>&);

  private:
        /// Rotation cycle of inner axle, Eigen value of each sensor (is displayed under the sensor back)
        unsigned Tx;
        /// Rotation cycle of outer axle, Eigen value of each sensor (is displayed under the sensor back)
        unsigned Ty;
        /// Pixel numbers horizontally
        unsigned pixNumHor;
        /// Pixel numbers vertically
        unsigned pixNumVert;

        // data blocks
        /// The data between the synchronization header and the ranging data of a data block.
        std::vector<unsigned char> afterHeader;
        /// The xy data of a data block.
        std::vector<unsigned char> xyData;
        /// The ranging data of a data block.
        std::vector<unsigned char> rangingData;
        /// The timestamp of a data block.
        std::vector<unsigned char> timestamp;
        /// The tail of a data block.
        std::vector<unsigned char> tail;

        // data
        /// The light volume data.
        std::vector<std::vector<float> > lightVolume;
        /// The distance data.
        std::vector<std::vector<float> > distance;
        /// The 3d position data.
        std::vector<std::vector<std::vector<std::vector<float> > > > position3d;

        // lookup tables for scan positions and times
        /// Lookup table for scan positions.
        std::vector<std::vector<std::vector<unsigned> > > posLookup;
        /// Loopup table for scan times.
        std::vector<std::vector<std::vector<float> > > dirLookup;

    /// The serial device;
        SerialDevice device;
        /// The current timestamp.
    Base::TimeStamp *currentTimeStamp;
    /// The frame rate in frame per second.
        unsigned char fps;

        /// The current command.
        unsigned char currentCmd;
        /// The current frame (that is 0 or 1)
        unsigned char currentFrame;
        /// The response code details of the last command response
        unsigned char responseCodeDetails;
        /// The response data details of the last command response
        unsigned char responseDataDetails;
        /// True, if the last CRC test is passed, false otherwise.
        bool CRC;

        /// Fills the position lookup table posLookup with the positions of the scanning points of the Lissajous figure.
    void getXYPositions(int frame);
        /// Calculates the directions of the Lissajous scanning points according to the formula in the insctruction manual.
    void calcDir(int frame, float t_clk, std::vector<float>& result);
    /// Calculates the 3d positions using the calculated lissajous scanning points positions and directions.
        void getRangingData(int frame);

};

#endif
