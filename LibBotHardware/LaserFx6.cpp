#include "LaserFx6.h"

#include "Crc16.h"
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <math.h>
#include <string.h>
#include <string>

#define SIN(x) sin( (x)*(M_PI) / 180.0 )
#define COS(x) cos( (x)*(M_PI) / 180.0 )
#define SIN_MINUS_21 -0.35836794954530027348413778941347
#define COS_MINUS_21 0.93358042649720174899004306313957
#define COS_48 0.66913060635885821382627333068678
#define SIN_48 0.74314482547739423501469704897426

#define POSITIVE_RESPONSE (readData() == char(0x81) && afterHeader[3] == 0x06)

// FX6 commands
// Command to start scanning
unsigned char startCmd[] =      { 0xff, 0xff, 0xff, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0xBA, 0x70, 0x04 };
// Command to stop scanning
unsigned char stopCmd[] =       { 0xff, 0xff, 0xff, 0x00, 0x01, 0x00, 0x00, 0x00, 0x09, 0x3B, 0x78, 0x04 };
// Command to set the framerate (Positionen 8-10: fps16 => 0x00, 0xF9, 0xC7 / fps=8 => 0x01, 0xE9, 0xE6)
unsigned char frameRateCmd[] =  { 0xff, 0xff, 0xff, 0x00, 0x01, 0x00, 0xCC, 0x00, 0x00, 0xF9, 0xC7, 0x04 };

LaserFx6::LaserFx6(unsigned tx, unsigned ty, unsigned pixnumhor, unsigned pixnumvert)
{
    Tx = tx;
    Ty = ty;
    pixNumHor = pixnumhor;
    pixNumVert = pixnumvert;

    // Allocating memory for the vectors
    afterHeader.resize(6);
    xyData.resize(pixNumHor * pixNumVert * 6);
    rangingData.resize(pixNumHor * pixNumVert * 6);
    timestamp.resize(2);
    tail.resize(3);

    lightVolume.resize(2);
    for (unsigned i=0; i!=2; ++i) {
        lightVolume[i].resize(pixNumHor * pixNumVert);
    }

    distance.resize(2);
    for (unsigned i=0; i!=2; ++i) {
        distance[i].resize(pixNumHor * pixNumVert);
    }

    position3d.resize(2);
    for (unsigned i=0; i!=2; ++i) {
        position3d[i].resize(pixNumHor);
        for (unsigned j=0; j!=pixNumHor; ++j) {
            position3d[i][j].resize(pixNumVert);
            for (unsigned k=0; k!=pixNumVert; ++k) {
                position3d[i][j][k].resize(3);
            }
        }
    }
    cout << "position3d sizes h="<<pixNumHor << " v="<<pixNumVert<<endl;
    posLookup.resize(2);
    for (unsigned i=0; i!=2; ++i) {
        posLookup[i].resize(pixNumHor * pixNumVert);
        for (unsigned j=0; j!=pixNumHor*pixNumVert; ++j) {
            posLookup[i][j].resize(2);
        }
    }

    dirLookup.resize(2);
    for (unsigned i=0; i!=2; ++i) {
        dirLookup[i].resize(pixNumHor * pixNumVert);
        for (unsigned j=0; j!=pixNumHor*pixNumVert; ++j) {
            dirLookup[i][j].resize(3);
        }
    }

  if (frameRateCmd[8] == 0x00) fps = 8;
  else fps = 16;

  currentFrame = 0;
  currentCmd = 0;
  currentTimeStamp = 0;
}

LaserFx6::~LaserFx6(void)
{
  stop();
  if(currentTimeStamp) delete currentTimeStamp;
}

bool LaserFx6::connect(const char* port)
{

  SERIAL_RESULT_TYPE resultType = device.connect(port);

  if(resultType == 0) {
    return true;
  } else {
    return false;
  }
}

bool LaserFx6::close()
{
  return device.close();
}

bool LaserFx6::start()
{
  if (device.isConnected() && stop()) {
        // There is no more than three tries to start the scanning process (three is a lucky number).
        for (unsigned i=0; i!=3; ++i) {
            device.send(startCmd, 12);
            if (POSITIVE_RESPONSE) return true;
        }
    }
    return false;
}

bool LaserFx6::stop()
{
    // There is no more than three tries to stop the scanning process (three is a lucky number).
    for (unsigned i=0; i!=3; ++i) {
        device.send(stopCmd, 12);
    if (POSITIVE_RESPONSE) return true;
    }
  return false;
}

char LaserFx6::readData()
{
    cout << "reading data" <<endl;
  // find sync bits
  if(currentTimeStamp) delete currentTimeStamp;
  currentTimeStamp = new Base::TimeStamp();

  int count = 0;
    int count_sum = 0;
  unsigned char byte[1];
  while(count != 3) {
    device.receive(byte, 1);
    cout << "received byte"<<endl;
    if(byte[0] == 0xff) {
      count++;
    }
        else {
      count = 0;
    }
        ++count_sum;
        // There is no more than three tries to find the sync bits (three is a lucky number).
        if (count_sum > 3) {
            throw ("Could not find synchronization bits");
        }
  }

  device.receive(&afterHeader[0], afterHeader.size());
  printf("received by te %x\n",afterHeader[1]);
  currentCmd = afterHeader[1];
    std::string datastream;

    switch(afterHeader[1]) {
    case 0x81: // Response Command
            device.receive(&tail[0], tail.size());
            responseCodeDetails = afterHeader[3];
            responseDataDetails = afterHeader[4];
            datastream = bytesToBits(afterHeader) + bytesToBits(tail, tail.size()-1);
            CRC = checkCRC16(datastream);
            break;

    case 0x83: // XY data
            device.receive(&xyData[0], pixNumHor * pixNumVert * 6);
      device.receive(&timestamp[0], timestamp.size());
            device.receive(&tail[0], tail.size());
            getXYPositions(afterHeader[3]);
            currentFrame = afterHeader[3];
            datastream = bytesToBits(afterHeader) + bytesToBits(xyData)
                                 + bytesToBits(timestamp) + bytesToBits(tail, tail.size()-1);
            CRC = checkCRC16(datastream);
            if (!CRC) throw ("Invalid CRC in XY data");
        break;

        case 0x85: // Ranging data
            device.receive(&rangingData[0], pixNumHor * pixNumVert * 6);
      device.receive(&timestamp[0], timestamp.size());
            device.receive(&tail[0], tail.size());
            getRangingData(afterHeader[3]);
      currentFrame = afterHeader[3];
            datastream = bytesToBits(afterHeader) + bytesToBits(rangingData)
                                 + bytesToBits(timestamp) + bytesToBits(tail, tail.size()-1);
            CRC = checkCRC16(datastream);
      break;

        default:
            CRC = false;
  }

  return afterHeader[1];
}

bool LaserFx6::setFPS(unsigned char fps)
{
  this->fps = fps;
  if(fps < 16) {
    frameRateCmd[8] = 0x01;
        frameRateCmd[9] = 0xE9;
        frameRateCmd[10] = 0xE6;
  } else {
    frameRateCmd[8] = 0x00;
        frameRateCmd[9] = 0xF9;
        frameRateCmd[10] = 0xC7;
  }

  if(stop()) {
        // There is no more than three tries to send the frame rate command (three is a lucky number).
        for (unsigned i=0; i!=3; ++i) {
            device.send(frameRateCmd, 12);
            if (POSITIVE_RESPONSE) return true;
        }
  }
    return false;
}

void LaserFx6::getXYPositions(int frame) {
  int time = 0;
  for(unsigned i = 0; i < pixNumHor * pixNumVert; ++i) {
    // set values for the time lookup table
    time = time + ( ( ((int)xyData[i*6 + 1]) << 16) | ( ((int)xyData[i*6 + 2]) << 8) | ((int)xyData[i*6 + 3]) );

    // set values for the position lookup table
    posLookup[frame][i][0] = (int)xyData[i*6 + 4];
    posLookup[frame][i][1] = (int)xyData[i*6 + 5];

    // Calc the direction
    calcDir(frame, (float)time, dirLookup[frame][i]);
  }
}

void LaserFx6::calcDir(int frame, float t_clk, std::vector<float>& result)
{
  float phi_x = -12.5f * (float)sin( 2.0f*M_PI * (t_clk/(2.0f*Tx)  )  - (M_PI/2.0f));
  float phi_y = 15.0f * (float)sin( 2.0f*M_PI * (t_clk/(2.0f*Ty) ) - (M_PI/2.0f));

  float n_x = (float)(SIN( phi_y ) * COS( phi_x ));
  float n_y = (float)(-SIN( phi_x ) * COS_MINUS_21 - COS( phi_x ) * COS( phi_y ) * SIN_MINUS_21);
  float n_z = (float)(-SIN( phi_x ) * SIN_MINUS_21 + COS( phi_x ) * COS( phi_y ) * COS_MINUS_21);

  float n = (float)(-2.0f * (-COS_48 * n_y - SIN_48 * n_z ));

  if(frame == 0) {
    result[0] = n * n_x;
  }
    else {
        result[0] = -n * n_x;
  }
  result[1] = (float)(-COS_48 + n * n_y);
  result[2] = (float)(-SIN_48 + n * n_z);
}

void LaserFx6::getRangingData(int frame)
{
    int rang = 0;
    for(unsigned i=0; i < pixNumHor * pixNumVert; ++i) {

    rang = (int)rangingData[i*6 + 4];
    rang = rang << 8 | (int)rangingData[i*6 + 5];
    distance[frame][i] = ((float)(0xFFF & rang))*(4.0f/1000.0f);
    lightVolume[frame][i] = ((((int)rangingData[i*6]) << 4) | ( ((int)rangingData[i*6+2])) | (((int)rangingData[i*6+4]) >> 4)) / 4096.0f;

    position3d[frame][ posLookup[frame][i][0] ][ posLookup[frame][i][1] ][0] = dirLookup[frame][ i ][0] * distance[frame][ i ];
    position3d[frame][ posLookup[frame][i][0] ][ posLookup[frame][i][1] ][1] = dirLookup[frame][ i ][1] * distance[frame][ i ];
    position3d[frame][ posLookup[frame][i][0] ][ posLookup[frame][i][1] ][2] = dirLookup[frame][ i ][2] * distance[frame][ i ];
  }

}

void LaserFx6::getLightVolume(vector<float>& lv)
{
    for (unsigned i=0; i!=pixNumHor * pixNumVert; ++i) {
        unsigned index = posLookup[currentFrame][i][0] + posLookup[currentFrame][i][1] * pixNumHor;
        lv[index] = lightVolume[currentFrame][i];
    }
}

void LaserFx6::getDistance(float* dist)
{
    for (unsigned i=0; i!=pixNumHor * pixNumVert; ++i) {
        unsigned index = posLookup[currentFrame][i][0] + posLookup[currentFrame][i][1] * pixNumHor;
        dist[index] = distance[currentFrame][i];
    }
}

void LaserFx6::getPosition(vector<float>& pos)
{
    unsigned i=0;
    for (unsigned h=0; h!=pixNumVert; ++h) {
        for (unsigned w=0; w!=pixNumHor; ++w) {
            pos[i++] = position3d[currentFrame][w][h][0];
            pos[i++] = position3d[currentFrame][w][h][1];
            pos[i++] = position3d[currentFrame][w][h][2];
        }
    }
}

