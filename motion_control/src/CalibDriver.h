#ifndef CALIBDRIVER_H
#define CALIBDRIVER_H
#include <ros/ros.h>
#include <utils/LibRobot/LaserEnvironment.h>
#include "StatsEstimator.h"
#include <utils/LibOdoCalib/EncoderEstimator.h>
#include <utils/LibOdoCalib/BetaEstimator.h>
#include "Stopwatch.h"
#include "MotionController.h"
#include <utils/LibOdoCalib/DualAxisCalib.h>

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
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CalibDriver(ros::Publisher& cmd_pub,ros::NodeHandle& node);
    virtual void start ();
    virtual void stop ();
    virtual int getType () {
      return motion_control::MotionGoal::MOTION_ODO_CALIB;
    }
    virtual int execute (MotionFeedback& fb, MotionResult& result);
    virtual void configure (ros::NodeHandle &node);
    virtual void setGoal (const motion_control::MotionGoal& goal);

private:
    void publish ();
    int doStartMove(MotionFeedback& fb, MotionResult& result);
    int doCtrlDrive(MotionFeedback& fb, MotionResult& result);

    double  calcBetaAngle(const Vector3d& p1, const Vector3d& p2, int direction);

    double  calcLsBetaAngle(Vector2dVec& ps, double theta,int direction);

    ros::Publisher&     cmd_pub_;
    int                 state_;

    ICtrl               dual_axis_calib_;

    float               speed_, speed_zero_;
    LaserEnvironment    laser_env_;
    double              min_delta_dist_;
    int                 noise_front_, noise_rear_;
    double              tuning_a_;
    int                 servo_front_fixed_,servo_rear_fixed_;
    int                 servo_front_mid_,servo_rear_mid_;
    int                 servo_front_start_,servo_rear_start_;
    float_t              beta_interval_;
    int                 interval_threshold_;
    int                 sleep_time_ms_;
    int                 controlled_count_;

    // estimators
    StatsEstimator<float> servo_front_stats_,servo_rear_stats_;
    vector<StatsEstimator<float> > hall_stats_;
    StatsEstimator<double> beta_err_stats_,yaw_err_stats_, yawrate_err_stats_;
    EncoderEstimator    encoder_estimator_;
    BetaEstimator       beta_estimator_;

    // limits for fail detection and thresholds
    double              max_beta_wait_time_;
    double              max_drive_distance_,min_drive_distance_;
    int                 ctrl_sleep_ms_; // time between controller executions
    // timers and current measurements
    Stopwatch           move_timer_;
    Stopwatch           ctrl_timer_;
    Vector3d            start_pose_;
    double              dist_driven_;

    // controller set values
    double              beta_target_;
    double              theta_target_;

    // command vector
    double              cmd_v_;
    float              cmd_servof_,cmd_servor_;

};

#endif // CALIBDRIVER_H
