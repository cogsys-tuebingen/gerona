/**************************************************************************
    @project RA Outdoor Robot System
    @author Karsten Bohlmann
    @date 5/5/2010 early 21st century
    (c) Universitaet Tuebingen 2010

**************************************************************************/
#include <math.h>
#include "SinousDriver.h"

SinousDriver::SinousDriver(PlayerClient *robot, CircleEstimator *circleEstimator)
    :mRobot(robot),mCircleEstimator(circleEstimator)
{
    mSpeed = 0.4;
    mSteerDegStep = 5;
    mSteerDeg = mSteerDegStep;
    mMaxSteerDeg = 30;
    mArcThreshold = 0.8;
}


bool SinousDriver::Update ()
{
    double arcLength = mCircleEstimator->GetArcLength();
    std::cout<<"arclen"<<arcLength<<std::endl;
    if (arcLength>mArcThreshold) {
        // done driving this segment
        if (fabs(mSteerDeg)<fabs(mMaxSteerDeg)) {
            if (mSteerDeg<0) {
                mSteerDeg=mSteerDeg*-1.0;
                mSteerDeg+=mSteerDegStep;
            } else {
                // switch from left to right turn
                mSteerDeg=mSteerDeg*-1.0;
            }

            std::cout << "new steer angle "<< mSteerDeg << std::endl;
        } else {
            std::cout << "STOPPING"<< std::endl;
            mSpeed = 0;
            mSteerDeg=0;
        }
        return true;
    } else {
        return false;
    }
}


void SinousDriver::GetDriveParams(double& speed, double& steerAngleRad)
{
    speed = mSpeed;
    steerAngleRad = mSteerDeg*M_PI/180.0;
}
