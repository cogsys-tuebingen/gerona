#ifndef CALIBBOTINTERFACE_H
#define CALIBBOTINTERFACE_H
#include <Eigen/Core>
#include "Global.h"
using namespace Eigen;

class CalibBotInterface
{
public:

    /**
    set speed and front steering angle of robot
    */
  virtual void SetSpeedSteer(float speed, float steer )=0;
  virtual void SetSpeedSteer(float speed, float front_rad_, float rear_rad_ )=0;


  virtual void SetSpeedAndServos (float speed,float servoF, float servoR)=0;
  virtual void Read()=0;

  // Setter
  // Getter
  virtual bool GetOdometryPose( Vector3d &pose ) const =0;
  virtual void GetOdometryDelta( Vector3d &delta ) const =0;
  virtual bool GetSlamPose( Vector3d &pose ) const =0;
  virtual int GetFrontSteerServo() const =0;
  virtual int GetRearSteerServo() const =0;
  /**
    return the driven distance calculated as mean of left/right encoder distance
    ***todo fails for robots with one encoder
    */
  virtual double GetEncoderDistance() const=0;
  virtual int GetLeftEncoderTicks() const =0;
  virtual int GetRightEncoderTicks() const =0;
  virtual void GetLaserReadings(int laserPos,FVector& ranges, float& minAngle, float& maxAngle) =0;

  virtual double GetOdoSpeedFiltered() const =0;
  virtual double GetPniYaw () const =0;
  virtual bool IsFreshSlamPose() const =0;
  virtual bool IsFreshOdometryPose() const =0;
  virtual bool IsFreshEncoderData() const =0;
  virtual bool IsFreshLaserData (int laserPos) const =0;
  virtual bool IsFreshPniData () const =0;
  virtual bool IsProxiesOk() const =0;
  virtual void GetHallVoltages(DVector& volts) const = 0;

};

#endif // CALIBBOTINTERFACE_H
