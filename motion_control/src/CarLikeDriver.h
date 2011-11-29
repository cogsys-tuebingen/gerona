#ifndef CARLIKEDRIVER_H
#define CARLIKEDRIVER_H
class CarLikeDriver
{
public:
  virtual void Update( const Eigen::Vector3d &target ) = 0;
  virtual void GetCmd(double& speed, double& front_rad, double& rear_rad) const = 0;
  //virtual void SetSpeedFactor (double K_v) = 0;
};


#endif // CARLIKEDRIVER_H
