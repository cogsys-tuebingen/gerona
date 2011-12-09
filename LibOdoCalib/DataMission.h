#ifndef DataMission_H
#define DataMission_H

///////////////////////////////////////////////////////////////////////////////
// I N C L U D E S
///////////////////////////////////////////////////////////////////////////////

// Workspace
#include <ConfigFileReader.h>
#include <iostream>
#include <fstream>
// Project
#include "LaserEnvironment.h"
#include "OdoCalibration.h"
#include "ProxyWrapper.h"
#include "StraightLineDriver.h"
#include "EncoderEstimator.h"
#include "DualAxisCalib.h"
///////////////////////////////////////////////////////////////////////////////
// D E C L A R A T I O N S
///////////////////////////////////////////////////////////////////////////////

class DataMission : public CalibrationMission {
public:

    /**
     * Constructor.
     *
     * @param config Coniguration file.
     * @param proxy Communicates with the robot.
     */
    DataMission( ConfigFileReader *config, ProxyWrapper *proxy );

    /* Inherited from CalibrationMission */
    bool Execute();


private:
    void FindObstacles ();
    double CalcBeta (const Vector3d& p1, const Vector3d& p2);
    void LogGnuPlotData (double deltaT,const Vector3d& poseOld, const Vector3d& poseNew, double gmYaw);
    /** Communicates with the robot. */
    ProxyWrapper *mProxy;
    /** Used to drive a straight line */
    StraightLineDriver mLineDriver;
    /** Computes the encoder calibration */
    EncoderEstimator mEncEst;
    /** Number of calibration runs */
    int mNumRuns;
    /** Minimum distance */
    double mMinDist;
    /** Mission speed [m/s] */
    double mSpeed;
    /** min distance the robot moved between updates */
    double mMinDelta;
    int mNoiseFront, mNoiseRear;
    Stopwatch mMissionTimer;
    LaserEnvironment mLaserEnv;
    DualAxisCalib mDualAxisCalib;
    ofstream mGnuLog;
};

#endif // DataMission_H
