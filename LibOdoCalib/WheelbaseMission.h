#ifndef WHEELBASEMISSION_H
#define WHEELBASEMISSION_H

#include "OdoCalibration.h"
#include "RobotProxy.h"
#include "CircleEstimator.h"
#include "SpiralDriver.h"
#include "KalmanWheelbase.h"

class WheelbaseMission : public RobotMission
{
public:
    WheelbaseMission( RobotProxy *proxy, ConfigFileReader& config );

    void Setup( ConfigFileReader& config );

    /* Inherited from CalibrationMission */
    bool Execute();

    /* Inherited from CalibrationMission */
    bool Done();

private:
    bool            mComplete;
    RobotProxy      *mRobot;
    CircleEstimator *mCircleEstimator;
    SpiralDriver    *mDriveAlgo;
    KalmanWheelbase *mKalman;
    double          mSteerRad;
};

#endif // WHEELBASEMISSION_H
