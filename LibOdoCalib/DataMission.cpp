
///////////////////////////////////////////////////////////////////////////////
// I N C L U D E S
///////////////////////////////////////////////////////////////////////////////

// C/C++
#include <iostream>
#include <cmath>

// Project/workspace
#include <MathHelper.h>
#include "LowPassFilter.h"
#include "DataMission.h"
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

DataMission::DataMission( ConfigFileReader *config, ProxyWrapper *proxy )
    : mProxy( proxy ), mLineDriver( proxy ), mLaserEnv(1)
{
    // Read config
    mNumRuns = config->GetInt( "DataMission::numberOfRuns", 4 );
    mMinDist = config->GetDouble( "DataMission::minDistance", 2.5 ); //3 );
    mSpeed = fabs( config->GetDouble( "DataMission::speedServo", 2350 ));
    double tuningA = config->GetDouble( "DataMission::tuningA", 1);
    mMinDelta = config->GetDouble("DataMission::minDelta",0.03);
    mNoiseFront = config->GetInt("DataMission::noiseFront",0);
    mNoiseRear = config->GetInt("DataMission::noiseRear",0);
    mDualAxisCalib.SetTuning(tuningA);

}


void DataMission::LogGnuPlotData(double deltaT, const Vector3d& p1, const Vector3d& p2, double gmYaw)
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



double DataMission::CalcBeta(const Vector3d& p1, const Vector3d& p2)
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



bool DataMission::Execute() {
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
    mProxy->SetCarlike(0.4,0.2);

    mGnuLog.open("dataMission.log");
    mGnuLog<< "%minDelta = "<<mMinDelta<< endl;
    mGnuLog<< "% t v yawRate beta x y yaw pniyaw"<<endl;

    Vector3d a,b;
    a << 0,0,M_PI/4;
    b << 1,0,0.3;
    double beta1=CalcBeta(a,b);
    cout << "betatest="<<beta1<< endl;
    cout << flush << endl;
    mProxy->GetSlamPose( slamPose );
    LowPassFilter<double> pniFilter(5);
    for (int i=0;i<15;++i) {
        mProxy->Read();
        if (!mProxy->IsFreshPniData()) {
            usleep(1500);
            continue;
        }
        double pniYaw=mProxy->GetPniYaw();
        pniFilter.Update(pniYaw);
    }

    double pniOffset = M_PI-pniFilter.GetValue();
    cout << "starrvalue pni="<<pniOffset*180.0/M_PI <<endl;
    mProxy->GetSlamPose( newPose );
    double gmOffset = (newPose(2)+slamPose(2))/2.0;
    cout << "startvalue gm="<<gmOffset*180.0/M_PI << endl;
    double yaw=0.0,newYaw = 0.0;
    mMissionTimer.restart();
    LowPassFilter<float> meanServoF(5);
    LowPassFilter<float> meanServoR(5);
    while (true) {
        mProxy->Read(); // Read incoming messages

        if (mProxy->IsFreshPniData()) {
            double pniYaw=mProxy->GetPniYaw();
            pniFilter.Update(pniYaw);
        }
        if ( !mProxy->IsFreshSlamPose()) {
            usleep( 1500 ); // Wait 1 msec
            continue;
        }
        if (mProxy->IsFreshLaserData()) {
            DVector laserRanges;
            double minAngle,maxAngle;
            mProxy->GetLaserReadings(laserRanges,minAngle,maxAngle);
            mLaserEnv.ProcessFrontLaserScan(laserRanges,minAngle,maxAngle);
            double minDist=mLaserEnv.GetMinDist();
            if (minDist<0.5) {
                mProxy->SetCarlike(0.0,0.0);
                cout << "Stopping"<< endl;
                continue;
            }
        }
        // New SLAM pose available
        mProxy->GetSlamPose( newPose );
        double gmYaw = newPose(2);
        newYaw = M_PI-pniFilter.GetValue()-pniOffset+gmOffset;
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
        mDualAxisCalib.Execute(ms/1000.0,beta,yaw,servoF,servoR);
        servoF+=mNoiseFront;
        servoR+=mNoiseRear;
        cout << "servoF:"<<servoF << " servoR:"<<servoR<< endl;
        mProxy->SetServos(mSpeed,servoF,servoR);
        meanServoF.Update(servoF);
        meanServoR.Update(servoR);
        if (meanServoF.IsLoaded()) {
            cout << "meanservoF:"<<meanServoF.GetValue()<< " meanservoR:"<<meanServoR.GetValue()<< endl;
        }
    }
/*
    // Compute and print average etc
    double leftMean = mean1D( leftCalib );
    double rightMean = mean1D( rightCalib );
    double leftStdDev = standardDeviation1D( leftCalib );
    double rightStdDev = standardDeviation1D( rightCalib );
*/
    return true;

}


void DataMission::FindObstacles ()

{

}

