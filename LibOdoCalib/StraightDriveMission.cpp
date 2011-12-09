
///////////////////////////////////////////////////////////////////////////////
// I N C L U D E S
///////////////////////////////////////////////////////////////////////////////

// C/C++
#include <iostream>
#include <cmath>

// Project/workspace
#include <MathHelper.h>
#include "LowPassFilter.h"
#include "StraightDriveMission.h"
#include "EncoderEstimator.h"
#include "Stopwatch.h"
#include "Misc.h"
// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
using namespace Eigen;


///////////////////////////////////////////////////////////////////////////////
// I M P L E M E N T A T I O N
///////////////////////////////////////////////////////////////////////////////

using namespace std;

StraightDriveMission::StraightDriveMission( ConfigFileReader *config, ProxyWrapper *proxy )
    : mProxy( proxy )
{
    mLaserEnvs.resize(2);
    // Read config
    mNumRuns = config->GetInt( "DataMission::numberOfRuns", 4 );
    mMinDist = config->GetDouble( "DataMission::minDistance", 2.5 ); //3 );
    mSpeed = fabs( config->GetDouble( "DataMission::speedServo", 2350 ));
    double tuningA = config->GetDouble( "DataMission::tuningA", 1);
    mMinDelta = config->GetDouble("DataMission::minDelta",0.03);
    mNoiseFront = config->GetInt("DataMission::noiseFront",0);
    mNoiseRear = config->GetInt("DataMission::noiseRear",0);


}


void StraightDriveMission::LogGnuPlotData(double deltaT, const Vector3d& p1, const Vector3d& p2, double gmYaw)
{
    if (deltaT<1e-3) {
        return;
    }
    double ds = (p2.head<2>()-p1.head<2>()).norm();
    double v=ds/deltaT; // speed in m/sec
    if (v<0.2) {
       return;
    }
    double beta = CalcBeta(p1,p2);

    double psi2=p2(2);
    double psi1=p1(2);
    Misc::normalizeAngle(psi2);
    Misc::normalizeAngle(psi1);
    double deltaYaw=psi2-psi1;
    Misc::normalizeAngle(deltaYaw);
    double yawRate = deltaYaw/(ds);
    double deltaX = p2(0)-p1(0);
    double deltaY = p2(1)-p1(1);

    if (deltaX<0.01) deltaX=0;
    if (deltaY<0.01) deltaY=0;

    mGnuLog << mMissionTimer.msElapsed()<<" " <<ds << " "<< (yawRate)<<" " << beta
            << " "<<p2.x()<< " "<<p2.y() <<" " << (psi2)<<" "<<gmYaw<< endl;
}



double StraightDriveMission::CalcBeta(const Vector3d& p1, const Vector3d& p2)
{

    //cout << "p1=" <<p1 <<endl;
    Vector2d A=p1.head<2>();
    Vector2d B=p2.head<2>();
    double theta = p1(2);
    double d = (B-A).norm();
    if (fabs(d)<1e-4) {
        return 0;
    }
    Vector2d dir;
    dir << cos(theta),sin(theta);
    Vector2d C=A+d*dir;
    Matrix3d T;
    T << A,B,C,1,1,1;
    //cout<< "A="<<A << endl;
    //cout <<" B="<<B <<endl;
    //cout << " C="<<C << endl;
    //cout << "T="<<T << endl;


    //double area=0.5*T.determinant();

    double h=T.determinant()/d;
    return h*-1.0;
}



bool StraightDriveMission::execute() {
    Vector3d slamPose,newPose;
    vector<double> leftCalib;
    vector<double> rightCalib;
    mProxy->Read(); // Read incoming messages
    mProxy->GetSlamPose( slamPose );
    Stopwatch timer;
    timer.restart();
    double yawRateSum = 0;
    double betaSum = 0;
    int n = 0;

    // get starting orientation values for slam and pni
    mProxy->GetSlamPose( slamPose );
    StatsEstimator<double> pniStats,slamStats;
    LowPassFilter<double> pniFilter(5);
    for (int i=0;i<15;++i) {
        mProxy->Read();
        if (mProxy->IsFreshPniData()) {
            pniStats.update(mProxy->GetPniYaw());
            pniFilter.Update(mProxy->GetPniYaw());
        }
        if (mProxy->IsFreshSlamPose()) {
            mProxy->GetSlamPose( newPose );
            slamStats.update(newPose(2));
        }
        usleep(10000);
    }

    double pniOffset = pniStats.getMean();
    Misc::normalizeAngle(pniOffset);
    cout << "startvalue pni="<<pniOffset*180.0/M_PI <<" sigma="<<pniStats.getStd()*180.0/M_PI<<endl;
    double gmOffset = slamStats.getMean();
    cout << "startvalue slam="<<gmOffset*180.0/M_PI << " sigma="<<slamStats.getStd()*180.0/M_PI<<endl;
    double yaw=0.0,newYaw = 0.0;
    mMissionTimer.restart();
    LowPassFilter<float> meanServoF(5);
    LowPassFilter<float> meanServoR(5);

    LaserEnvironment laserEnv(40);
    while (true) {
        mProxy->Read(); // Read incoming messages

        if (mProxy->IsFreshPniData()) {
            double pniYaw=mProxy->GetPniYaw();
            pniFilter.Update(pniYaw);
        }
        if ( !mProxy->IsFreshSlamPose()) {
            usleep( 1500 ); // Wait 1.5 msec
            continue;
        }
        if (mProxy->IsFreshLaserData(PROXY_FRONTLASER)) {
            DVector laserRanges;
            double minAngle,maxAngle;
            mProxy->GetLaserReadings(PROXY_FRONTLASER,laserRanges,minAngle,maxAngle);
            mLaserEnvs[PROXY_FRONTLASER].ProcessLaserScan(laserRanges,minAngle,maxAngle);
            double minDist=mLaserEnvs[PROXY_FRONTLASER].GetMinDist();
            if (minDist<0.5) {
                mProxy->SetSpeedSteer(0.0,0.0);
                cout << "Stopping"<< endl;
                continue;
            }
            double bestCourse=0.0;
            double bestDist = 0.0;
            laserEnv.ProcessLaserScan(laserRanges,minAngle,maxAngle);
            laserEnv.FindLongestCorridor(0.4,bestCourse,bestDist,-M_PI/4.0,+M_PI/4.0);
            cout << "best course:"<<bestCourse*180.0/M_PI<< " bestDist="<<bestDist<< endl;
            usleep(100000);
            continue;
        }
        // New SLAM pose available
        mProxy->GetSlamPose( newPose );
        double gmYaw = newPose(2);
        newYaw = pniFilter.GetValue()-pniOffset+gmOffset;
        double deltaX = newPose(0)-slamPose(0);
        double deltaY = newPose(1)-slamPose(1);
        if (fabs(deltaX)<mMinDelta && fabs(deltaY)<mMinDelta) {
            // wait for better data
            continue;
        }
        int ms = timer.msElapsed();
        timer.restart();
        slamPose(2)=yaw;
        newPose(2)=newYaw;

        Misc::normalizeAngle(newYaw);
        LogGnuPlotData(ms/1000.0, slamPose,newPose,gmYaw);
        double beta = CalcBeta(slamPose,newPose);


        slamPose = newPose;
        yaw = newYaw;
        betaSum+=beta;
        ++n;
        double yawRateMean=yawRateSum/n;
        Misc::normalizeAngle(yawRateMean);
        double betaMean = betaSum/n;
        Misc::normalizeAngle(betaMean);
        //cout << "beta:"<<beta*180.0/M_PI << " beta mean="<<betaMean*180.0/M_PI << endl;
        //cout << "deltaT:"<<ms << " nearest obstacle:"<<mLaserEnv.GetMinDist() << endl;
        //cout << "yawrate:"<<yawRate*180.0/M_PI <<"yawRateSum:"<<yawRateSum*180.0/M_PI << endl;
        float servoF, servoR;
        double yawSet = 0.0;
        double betaSet = 0.0;
        mDualAxisCalib.Execute(ms/1000.0,betaSet,beta,yawSet,yaw,servoF,servoR);
        servoF+=mNoiseFront;
        servoR+=mNoiseRear;
        cout << "servoF:"<<servoF << " servoR:"<<servoR<< endl;
        mProxy->SetSpeedAndServos(mSpeed,servoF,servoR);
        meanServoF.Update(servoF);
        meanServoR.Update(servoR);
        mServoFStats.update(servoF);
        mServoRStats.update(servoR);
        if (meanServoF.IsLoaded()) {
            cout << "meanservoF:"<<meanServoF.GetValue()<< " meanservoR:"<<meanServoR.GetValue()<< endl;

        }

    }

    return true;

}


void StraightDriveMission::FindObstacles ()

{

}

