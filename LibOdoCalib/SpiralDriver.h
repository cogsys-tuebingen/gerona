/**************************************************************************
    @project RA Outdoor Robot System
    @author Karsten Bohlmann
    @date 4/6/2010 early 21st century
    (c) Universitaet Tuebingen 2010

**************************************************************************/

#ifndef SPIRALDRIVER_H
#define SPIRALDRIVER_H
#include <libplayerc++/playerc++.h>
#include "CircleEstimator.h"
using namespace PlayerCc;

class SpiralDriver
{
    enum {
        ASCEND,
        DESCEND
    };
public:
    SpiralDriver(PlayerClient *robot, CircleEstimator *circleEstimator);
    bool Update ();
    void GetDriveParams(double& speed, double& steerAngleRad);

private:

    PlayerClient *mRobot;
    double mMaxSteerDeg;
    double mSteerDegStep;
    double mSteerDeg;
    double mSpeed;
    double mArcThreshold;
    int    mState;
    CircleEstimator *mCircleEstimator;
};

#endif // SPIRALDRIVER_H
