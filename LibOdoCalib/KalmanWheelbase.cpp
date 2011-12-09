/**************************************************************************
    @project RA Outdoor Robot System
    @author Karsten Bohlmann
    @date 4/23/2010 early 21st century
    (c) Universitaet Tuebingen 2010

**************************************************************************/
#include <iostream>
#include <Eigen/LU>
#include "KalmanWheelbase.h"
using namespace std;

KalmanWheelbase::KalmanWheelbase(double wheelBase)
{
    x(0)=0;
    x(1)=wheelBase;
    P(0,0) = 0;
    P(0,1) = 0;
    P(1,0) = 0;
    P(1,1) = 0;
    SetMeanState(0,wheelBase);
}


void KalmanWheelbase::GetEstimation(Vector2d &res)
{
    res=x;
}

void KalmanWheelbase::Update(double zr,double zdelta)
{
    cout<<"radius:"<<zr << " angle(deg):" << zdelta*180/M_PI << endl;
    double delta = zdelta;
    double r = zr;
    double l = x(1);
    // skip if steering angle is approx 0
    if (fabs(delta)<1e-3) {
        return;
    }
    double cosd = cos(delta);
    double sind = sin(delta);
    double sind2 = sind*sind;
    double sind4 = sind2*sind2;

    Matrix2d Pp, K1,K2;
    Pp = Q;
    Pp(1,1)+=P(1,1);
    cout << "Pp:"<<Pp << endl;
    // kalman gain K = K1*inv(K2)
    K1(0,0)=-l*Q(0,0)*cosd/sind2;
    K1(0,1)=Q(0,0);
    K1(1,0)=(Pp(1,1)+Q(1,1))/sind;
    K1(1,1)=0;

    K2(0,0)=R(0,0)+(Pp(1,1)+Q(1,1))/sind2+(l*l*Q(0,0)*cosd*cosd)/sind4;
    K2(0,1)=-l*Q(0,0)*cosd/sind2;
    K2(1,0)=-l*Q(0,0)/sind2;
    K2(1,1)=Q(0,0)+R(1,1);
    Matrix2d K = K1*K2.inverse();

    double h1=l/sind;
    double h2=delta;
    Vector2d zh;
    zh(0)=zr-h1;
    zh(1)=zdelta-h2;
    Vector2d xnew=x+K*(zh);
    Matrix2d H;
    H(0,0)=-l*cosd/sind2;
    H(0,1)=1/sind;
    H(1,0)=1;
    H(1,1)=0;
    Matrix2d Pnew = (Matrix2d::Identity()-K*H)*Pp;
    x=xnew;
    P=Pnew;

}


void KalmanWheelbase::SetProcessVar(double steerQ,double wheelBaseQ)
{
    Q(0,0)=steerQ;
    Q(1,1)=wheelBaseQ;
    Q(1,0) = 0;
    Q(0,1) = 0;
}


void  KalmanWheelbase::SetMeasureVar(double radiusR, double steerR)
{
    R(0,0) = radiusR;
    R(1,1) = steerR;
    R(0,1) = 0;
    R(1,0) = 0;
}


void KalmanWheelbase::SetMeanState(double steerRad, double wheelBase)
{
    x(0)=steerRad;
    x(1)=wheelBase;

}
