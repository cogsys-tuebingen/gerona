/**************************************************************************
    @project RA Outdoor Robot System
    @author Karsten Bohlmann
    @date 5/5/2010 early 21st century
    (c) Universitaet Tuebingen 2010

**************************************************************************/

#ifndef SINOUSDRIVER_H
#define SINOUSDRIVER_H
#include <libplayerc++/playerc++.h>
#include "CircleEstimator.h"
using namespace PlayerCc;

class SinousDriver
{
public:
    SinousDriver(PlayerClient *robot, CircleEstimator *circleEstimator);
    bool Update ();
    void GetDriveParams(double& speed, double& steerAngleRad);

private:
    PlayerClient *mRobot;
    double mMaxSteerDeg;
    double mSteerDegStep;
    double mSteerDeg;
    double mSpeed;
    double mArcThreshold;
    CircleEstimator *mCircleEstimator;
};
#endif // SINOUSDRIVER_H
