/**************************************************************************
    @project RA Outdoor Robot System
    @author Karsten Bohlmann
    @date 4/6/2010 early 21st century
    (c) Universitaet Tuebingen 2010

**************************************************************************/

#include "SpiralDriver.h"

SpiralDriver::SpiralDriver (PlayerClient *robot, CircleEstimator *circleEstimator)
    :mRobot(robot),mCircleEstimator(circleEstimator)
{
    mSpeed = 0.4;
    mSteerDegStep = 5;
    mSteerDeg = mSteerDegStep;
    mMaxSteerDeg = 30;
    mArcThreshold = 0.8;
    mState = ASCEND;
}


bool SpiralDriver::Update ()
{
    double arcLength = mCircleEstimator->GetArcLength();
    if (arcLength>mArcThreshold) {
        if (mState==ASCEND) {

            if (mSteerDeg<mMaxSteerDeg) {
                mSteerDeg+=mSteerDegStep;
                std::cout << "new steer angle "<< mSteerDeg << std::endl;
            } else {
                mState=DESCEND;
                mSteerDeg-=mSteerDegStep;
            }
        } else {
            if (mSteerDeg>mSteerDegStep) {
                mSteerDeg-=mSteerDegStep;
                std::cout << "new steer angle "<< mSteerDeg << std::endl;
            } else {
                mState=ASCEND;
                mSteerDeg+=mSteerDegStep;
                mSpeed=0;
            }
        }
        return true;
    } else {
        return false;
    }
}


void SpiralDriver::GetDriveParams(double& speed, double& steerAngleRad)
{
    speed = mSpeed;
    steerAngleRad = mSteerDeg*M_PI/180.0;
}
