/**************************************************************************
    @project RA Outdoor Robot System
    @author Karsten Bohlmann
    @date 4/23/2010 early 21st century
    (c) Universitaet Tuebingen 2010

**************************************************************************/

#ifndef KALMANWHEELBASE_H
#define KALMANWHEELBASE_H
#include "Eigen/Core"
using namespace Eigen;


/**
  extended kalman filter estimating wheelbase of ackermann robot
  state vector x = [delta, l]
  measurement vector z = [r, delta]


  */
class KalmanWheelbase
{
public:
    KalmanWheelbase(double wheelBase);

    /**
      set variances of process noise
    */
    void SetProcessVar(double steerQ,double wheelBaseQ);

    /**
      set variances of process noise
    */
    void SetMeasureVar(double radiusR, double steerR);

    void SetMeanState(double steerRad, double wheelBase);
    void Update (double zr,double zdelta);
    void GetEstimation(Vector2d& res);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    Vector2d x; // state vector mean
    Matrix2d P; // state covariance

    Matrix2d R; // measurement covariance
    Matrix2d Q; // process covariance


};

#endif // KALMANWHEELBASE_H
