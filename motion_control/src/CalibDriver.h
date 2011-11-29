#ifndef CALIBDRIVER_H
#define CALIBDRIVER_H
#include <ros/ros.h>
#include "LaserEnvironment.h"
#include "StatsEstimator.h"
#include "EncoderEstimator.h"
#include "Stopwatch.h"
#include "MotionController.h"
#include "DualAxisCalib.h"

enum
{
  DIRECTION_FWD=0,
  DIRECTION_BWD=1
};
enum
{
  CALIB_STATE_STOP,
  CALIB_STATE_STARTMOVE,
  CALIB_STATE_CTRL
};

class CalibDriver : public MotionController
{
public:
    CalibDriver(ros::Publisher& cmd_pub);
    virtual void start ();
    virtual void stop ();
    virtual int getType () {
      return motion_control::MotionGoal::MOTION_ODO_CALIB;
    }
    virtual int execute ();
    virtual void setGoal (const motion_control::MotionGoal& goal);

private:
    void publish ();

    double  calcBetaAngle(const Vector3d& p1, const Vector3d& p2, int direction);

    double  calcLsBetaAngle(Vector2dVec& ps, double theta,int direction);



    int                 state_;
    ICtrl               dual_axis_calib_;
    double              beta_target_;
    float               speed_, speed_zero_;
    LaserEnvironment    laser_env_;
    double              min_delta_dist_;
    double              theta_target_;
    int                 noise_front_, noise_rear_;
    double              tuning_a_;
    int                 servo_front_fixed_,servo_rear_fixed_;
    int                 servo_front_mid_,servo_rear_mid_;
    int                 servo_front_start_,servo_rear_start_;
    StatsEstimator<float> servo_front_stats_,servo_rear_stats_;
    vector<StatsEstimator<float> > hall_stats_;
    StatsEstimator<double> beta_err_stats_,yaw_err_stats_, yawrate_err_stats_;
    float_t              beta_interval_;
    int                 interval_threshold_;
    double              max_drive_distance_,min_drive_distance_;
    int                 sleep_time_ms_;
    int                 ctrl_sleep_;
    int                 ctrl_cnt_;
    int                 controlled_count_;
    EncoderEstimator    encoder_estimator_;

    Stopwatch           move_timer_;
    Vector3d            start_pose_;
    ros::Publisher cmd_pub_;
    double cmd_v_;
    double cmd_servof_,cmd_servor_;

};

#endif // CALIBDRIVER_H
