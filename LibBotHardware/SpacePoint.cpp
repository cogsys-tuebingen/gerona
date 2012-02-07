#include "SpacePoint.h"

#include <iomanip>
#include <iostream>
#include <math.h>
#include <string.h>
#include <string>

SpacePoint::SpacePoint ()
{
    mHidMatcher.vendor_id = SPACEPOINT_VID;
    mHidMatcher.product_id = SPACEPOINT_PID;
    mHidMatcher.custom_data_length=0;
    mHidMatcher.matcher_fn=0;
    mHidMatcher.custom_data=0;
    mHid = 0;
    mIsConnected = false;
    Init ();
}


SpacePoint::~SpacePoint()
{
    Close();
    hid_delete_HIDInterface(&mHid);
    hid_cleanup();
}


bool SpacePoint::Init ()
{
    hid_return ret;

    // init hid subsystem if necessary
    if (!hid_is_initialised()) {
        ret = hid_init();
        if (ret != HID_RET_SUCCESS) {
            //player: PLAYER_ERROR( "Failed to initialise HID system" );
            return false;
        }
    }
    if (!mHid) {
        mHid = hid_new_HIDInterface();
        if (mHid == 0) {
            //player: PLAYER_ERROR( "Hid_new_HIDInterface() failed, out of memory?" );
            return false;
        }
    }
    return true;
}



bool SpacePoint::Connect()
{
    if (mIsConnected) {
        return true;
    }
    if (!mHid) {
        bool initOk = Init();
        if (!initOk) {
            return false;
        }
    }
    hid_return ret;

    ret = hid_force_open(mHid, 0, &mHidMatcher, 3);
    if (ret != HID_RET_SUCCESS) {
        return false;
    }
    mIsConnected = true;
    return true;
}


void SpacePoint::Close()
{
    if (mHid) {
        hid_close(mHid);
    }
    mIsConnected = false;
}


bool SpacePoint::ReadRawData(IVector& gyrosXYZ, IVector& mags)
{
    mags.resize(3);
    gyrosXYZ.resize(3);
    int words[10];
    U8 buffer[SPACEPOINT_LENGTH_RAW+1];
    int timeoutMs = 20; // 20msec timeout where farmerate of device is 125Hz
    hid_return ret = hid_interrupt_read(mHid,SPACEPOINT_ENDPOINT_RAW,(char *)buffer,SPACEPOINT_LENGTH_RAW,timeoutMs);
    if (ret!=HID_RET_SUCCESS) {
        //cout << "PNI SpacePoint: Read failed with code "<<ret <<endl;
        return false;
    }
    int idx=0;
    for (int i=0;i<10;++i) {
        words[i]=buffer[idx]+256*buffer[idx+1];
        idx+=2;
    }
    mags[0]=words[0];mags[0]=words[2];mags[2]=words[2];
    gyrosXYZ[0] = words[7]; // spacepoint gyro roll is player gyro X-axis
    gyrosXYZ[1] = words[8]; // spacepoint gyro pitch is player gyro Y-axis
    gyrosXYZ[2] = words[9]; // spacepoint gyro yaw is player gyro Z-axis
    return true;
}


bool SpacePoint::ReadFilteredData(FVector& accels, FVector& quat)
{
    int words[8];
    accels.resize(3);
    FVector quatDev(4);
    quat.resize(4);

    int timeoutMs = 20; // 20msec timeout where farmerate of device is 125Hz
//    hid_return ret = hid_interrupt_read(mHid,SPACEPOINT_ENDPOINT_FILTERED,(char *)words,SPACEPOINT_LENGTH_FILTERED,timeoutMs);
    U8 buffer[SPACEPOINT_LENGTH_FILTERED+1];
    hid_return ret = hid_interrupt_read(mHid,SPACEPOINT_ENDPOINT_FILTERED,(char *)buffer,SPACEPOINT_LENGTH_FILTERED,timeoutMs);
    if (ret!=HID_RET_SUCCESS) {
        //cout << "PNI SpacePoint: Read failed with code "<<ret <<endl;
        return false;
    }
    int idx=0;
    for (int i=0;i<7;++i) {
        words[i]=buffer[idx]+256*buffer[idx+1];
        idx+=2;
    }

    float values[8];
    for (int i=0;i<3;++i) {
        // see spacefusion application note for value calculation
        accels[i]=((words[i]-32768)*6.0)/32768.0;
    }
    for (int i=3;i<7;++i) {
        // see spacefusion application note for value calculation
        quatDev[i-3]=((words[i]-32768)*1.0)/32768.0;
    }
/*    quat[0]=quatDev[1];
    quat[1]=quatDev[2]*-1.0;
    quat[2]=quatDev[0];
    quat[3]=quatDev[3]*-1.0;
*/
    for (int i=0;i<4;++i) quat[i]=quatDev[i];

    return true;
}

