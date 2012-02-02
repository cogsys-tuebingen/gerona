#include <iostream>
#include "Global.h"
#include "Misc.h"
#include "DualAxisCalib.h"

ICtrl::ICtrl()
    :servos(2)
{
    mServoMin = 1500;
    mServoMax = 3000;
    KIyaw = 0.1;
    KPyaw = 1;
    Matrix2d k;
    k<<1,0.25,1,-0.175;
    SetKSinv(k);
    CalcController(1,1);
    Reset();
}


ICtrl::~ICtrl()
{
    // nothing to do
}


void ICtrl::SetServoDelta(int pos, float delta2u)
{
    servos[pos].delta2u=delta2u;
}


void ICtrl::SetServoMid(int pos, float mid)
{
    servos[pos].mid=mid;
}


void ICtrl::CalcController(double v, double a)
{
    a_ = a;
    if (v>0) {
        direction_=+1.0;
    } else {
        direction_=-1.0;
    }
    if (v<0) {
        KSinv_(0,1)=KSinv_(0,1)*-1.0;
        KSinv_(1,1)=KSinv_(1,1)*-1.0;
    }
    /*
    if (v>0) {
        // values calculated for v=1m/sec
        KSinv_ << 1 , 0.25,
                 1 , - 0.175 ;
    } else {
        // values calculated for v=-1m/sec
        KSinv_ << 1 , -0.25,
                 1 ,  0.175 ;
    }*/
    KI_ = a_*KSinv_;
    cout << "KSinv="<<KSinv_ << endl;
    Reset();
}


void ICtrl::Reset()
{
    yaw_int_ = 0;
    beta_int_ = 0.0;
}


void ICtrl::Execute(double deltaTSec, double betaSet, double beta, double yawSet, double yawIs, float& servoF,
                            float& servoR)
{

    // integrate
    double psiRateSet = 0.0;
    double betaDiff=Misc::normalizeAngle(betaSet-beta);


    beta_int_+=deltaTSec*(betaDiff);
    //mPsiRateInt+=deltaTSec*(psiRateSet - psiRate);
    Vector2d eInt;
    //eInt << mBetaInt,mPsiRateInt;
    double yawDiff = Misc::normalizeAngle(yawSet - yawIs);
    yaw_int_ +=KIyaw*deltaTSec*(yawDiff);
//    cout << "betaint="<<beta_int_ << " yawis"<<yawIs*180.0/M_PI<< " servomid="<<servos[0].mid<< "yawset="<<yawSet*180.0/M_PI
//         << "deg betadiff="<<betaDiff*180.0/M_PI <<"deg"<< endl;

    //eInt << beta_int_,yawDiff;
    eInt << beta_int_,(yaw_int_+KPyaw*yawDiff);
    Vector2d u = KI_*eInt;
    servoF=u(0)*servos[0].delta2u+servos[0].mid;
    servoR = u(1)*servos[1].delta2u+servos[1].mid;

   // cout << "u(0)"<<u(0)<< "servoF:"<<servoF  << " u1"<<u(1)<< " servoR"<<servoR<< endl;
    if (servoF<mServoMin) servoF=mServoMin;
    if (servoR<mServoMin) servoR=mServoMin;
    if (servoF>mServoMax) servoF=mServoMax;
    if (servoR>mServoMax) servoR=mServoMax;
}

