#ifndef SIMPLEGOALDRIVER_H
#define SIMPLEGOALDRIVER_H
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <utils/LibRobot/LaserEnvironment.h>
#include "StatsEstimator.h"
#include "Stopwatch.h"
#include "MotionController.h"
#include "DualPidCtrl.h"

class SimpleGoalDriver : public MotionController
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SimpleGoalDriver(ros::Publisher& cmd_pub,ros::NodeHandle& node);
    virtual void start ();
    virtual void stop ();
    virtual int getType () {
      return motion_control::MotionGoal::MOTION_TO_GOAL;
    }
    virtual int execute (MotionFeedback& fb, MotionResult& result);
    virtual void configure (ros::NodeHandle &node);
    virtual void setGoal (const motion_control::MotionGoal& goal);

    void updatePath (const nav_msgs::PathConstPtr& path);


private:
    void publish ();
    void predictPose (double dt, double deltaf, double deltar, double v,
                      Vector2d& front_pred, Vector2d& rear_pred);
    int driveToGoal (const Vector3d& goal,MotionFeedback& fb, MotionResult& result);
    double calculateCourse( double delta_f, double delta_r, double cmd_v );

    ros::NodeHandle&    node_handle_;
    ros::Publisher&     cmd_pub_;
    ros::Subscriber     path_subscriber_;
    double cmd_v_;
    double cmd_front_rad_,cmd_rear_rad_;
    double theta_target_;
    double default_v_;
    Vector2d pos_target_;
    int state_;
    Stopwatch move_timer_;
    geometry_msgs::PoseStamped goal_pose_global_;
    double pos_tolerance_;
    double theta_tolerance_;
    nav_msgs::Path goal_path_global_;
    unsigned path_idx_;
    double goal_v_;
    Vector3d start_pose_;
    DualPidCtrl ctrl_;
    double L_; // wheelbase
    double Tt_; // system dead time / latency
    double delta_max_;
    tf::TransformListener listener_;
    Vector3d last_slam_pose_;
};

#endif // SIMPLEGOALDRIVER_H
