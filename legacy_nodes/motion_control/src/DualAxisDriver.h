#ifndef DUALAXISDRIVER_H
#define DUALAXISDRIVER_H

///////////////////////////////////////////////////////////////////////////////
// I N C L U D E S
///////////////////////////////////////////////////////////////////////////////

// C/C++
#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <ros/ros.h>
#include <tf/tf.h>
// Eigen
#include "Line2d.h"
#include "CarLikeDriver.h"
#include "DualPidCtrl.h"

///////////////////////////////////////////////////////////////////////////////
// D E C L A R A T I O N S
///////////////////////////////////////////////////////////////////////////////

class DualAxisDriver : public CarLikeDriver {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum DriverState { DRIVE_STRAIGHT, DRIVE_STOP };

    DualAxisDriver(ros::NodeHandle& node);
    virtual void Update( const Eigen::Vector3d &target );
    virtual void GetCmd(double& speed, double& front_rad, double& rear_rad) const;

    void configure (ros::NodeHandle& node);

private:

    bool getRobotPose(Vector3d& pose);
    void driveInRow( const Eigen::Vector3d &target );
    void predictPose (double dt, double deltaf, double deltar, double v,
                      Vector2d& front_pred, Vector2d& rear_pred);
    DualPidCtrl ctrl_;

    Eigen::Vector2d mCmd;
    DriverState mState;
    double default_v_, default_turn_v_;
    double K_v_;
    double cmd_v_;
    double cmd_front_rad_,cmd_rear_rad_;
    double delta_max_;
    bool mLeftTurn;

    Stopwatch mission_timer_;
    std::string log_fname_;
    std::ofstream log_stream_;

    double L_; // wheelbase
    double Tt_; // system dead time / latency
    tf::TransformListener *pose_listener_;

};

#endif // DUALAXISDRIVER_H
