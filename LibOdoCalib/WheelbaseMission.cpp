
#include "WheelbaseMission.h"
#include "Eigen/Core"

using namespace Eigen;


WheelbaseMission::WheelbaseMission( RobotProxy *proxy, ConfigFileReader& config )
    : mComplete( false ), mRobot( proxy )
{
    mCircleEstimator = new CircleEstimator ();
    mDriveAlgo = new SpiralDriver(mRobot->GetPlayerClient(),mCircleEstimator);
    mKalman = new KalmanWheelbase(0.5);
    mSteerRad = 0;
    Setup(config);

    mKalman->SetMeasureVar(0.1,1*M_PI/180.0);
    mKalman->SetProcessVar(1*M_PI/180.0,0.1);
}

void WheelbaseMission::Setup(ConfigFileReader &config)
{
    double steerMeasureVar= config.GetDouble("steerMeasureVar",(1*180.0/M_PI)*(1*180.0/M_PI));
    double steerVar= config.GetDouble("steerVar",(1*180.0/M_PI)*(1*180.0/M_PI));
    double wheelbaseVar = config.GetDouble("wheelbaseVar",0.005*0.005);
    double radiusMeasureVar = config.GetDouble("radiusMeasureVar",0.05*0.05);
    double wheelbaseInit = config.GetDouble("wheelbaseInit",0.5);
    mKalman->SetMeasureVar(radiusMeasureVar,steerMeasureVar);
    mKalman->SetProcessVar(steerVar,wheelbaseVar);
    mKalman->SetMeanState(0,wheelbaseInit);
    std::cout << "wheelbas start="<<wheelbaseInit<< std::endl;
}

bool WheelbaseMission::Execute()
{
    Vector3f pose;
    mRobot->GetPose( pose );
    mCircleEstimator->AddPoint(pose[0],pose[1]);
    mCircleEstimator->Update();
    double  radius = mCircleEstimator->GetRadius();
    if (radius>0) {
        mKalman->Update(radius, mSteerRad);
        Vector2d est;
        mKalman->GetEstimation(est);
        std::cout << "Estimation: steerAngle:"<<est(0)*180.0/M_PI << " wheelbase:"<<est(1)<< std::endl;
    }

    bool arcCompleted = mDriveAlgo->Update();
    if (arcCompleted) {
        mCircleEstimator->Reset();
    }
    double speed, steerRad;
    mDriveAlgo->GetDriveParams(speed,steerRad);
    mRobot->SetCarlike(speed,steerRad);
    mSteerRad = steerRad;
    if (speed<1e-3) {
        mComplete = true;
    } else {
        mComplete = false;
    }
    return true;
}

bool WheelbaseMission::Done()
{
    return mComplete;
}
