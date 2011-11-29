#ifndef MOTIONCONTROLLER_H
#define MOTIONCONTROLLER_H
#include <Eigen/Core>
#include <tf/transform_listener.h>
#include "Global.h"
#include "motion_control/MotionGoal.h"
#include "motion_control/MotionFeedback.h"
using namespace Eigen;

class MotionController
{
public:
  virtual void start ()=0;
  virtual void stop ()=0;
  virtual int getType ()=0;
  virtual int execute ()=0;
  virtual void setGoal (const motion_control::MotionGoal& goal)=0;

  virtual bool getSlamPose(Vector3d& pose) const;
  virtual bool laserCallback(const sensor_msgs::LaserScanConstPtr& scan)

protected:
  tf::TransformListener pose_listener_;

};


#endif // MOTIONCONTROLLER_H
