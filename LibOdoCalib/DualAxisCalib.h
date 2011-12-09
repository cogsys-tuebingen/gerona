#ifndef DUALAXISCALIB_H
#define DUALAXISCALIB_H
#include <vector>
#include "Eigen/Core"
using namespace Eigen;


struct ServoSpec
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ServoSpec() {mid=2250;delta2u=(3000-2250)/(20.0*M_PI/180.0);}
    float mid;
    float delta2u;
};
typedef std::vector<ServoSpec> ServoSpecVec;
class ICtrl
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ICtrl();
    ~ICtrl();
    void Execute (double deltaTSec, double betaSet, double betaIs, double yawSet,double yawIs,
                  float& servoF,float& servoR);

    void Reset ();

    void CalcController (double v,double tuning_a);
    void SetServoMid (int servo,float mid);
    void SetServoDelta (int servo,float delta2u);
    void SetKIyaw (double ki) {KIyaw=ki;}

    void SetKPyaw (double kp) {KPyaw=kp;}

    void SetKSinv (const Matrix2d& k) {KSinv_=k;}
private:
    double beta_int_,yaw_int_;
    Matrix2d KI_,KSinv_;
    double KIyaw,KPyaw;
    ServoSpecVec servos;
    double a_,b_;
    double direction_;
    float mServoMin,mServoMax;
};

#endif // DUALAXISCALIB_H
