#ifndef ROBOTCONTROLLER_KINEMATIC_HBZ_TT_H
#define ROBOTCONTROLLER_KINEMATIC_HBZ_TT_H

/// PROJECT
#include <path_follower/controller/robotcontroller.h>
#include <path_follower/utils/parameters.h>
#include "robotcontroller_kinematic_HBZ.h"


class RobotController_Kinematic_HBZ_TT : public RobotController_Kinematic_HBZ
{
public:
    /**
     * @brief RobotController_Kinematic_HBZ_TT
     */
    RobotController_Kinematic_HBZ_TT();

protected:
    /**
     * @brief computeSpeed computes the actual command velocity, by using different speed control techniques
     *
     * Here, a special control speed term is added, when compared to the "kinematic_HBZ" in order to account
     * for the desired distance to the person.
     *
     */
    virtual double computeSpeed();

private:
    struct ControllerParameters : public RobotController_Kinematic_HBZ::ControllerParameters
    {
        P<double> des_dist;
        P<double> k_l;

        ControllerParameters():
            RobotController_Kinematic_HBZ::ControllerParameters("kinematic_hbz_tt"),
            des_dist(this, "des_dist", 2.0, "Desired distance for target tracking."),
            k_l(this, "k_l", 1.0, "Factor for regulating the speed control for target tracking.")
        {}
    } opt_;

    const RobotController::ControllerParameters& getParameters() const
    {
        return opt_;
    }

    //previous time stamp
    ros::Time previous_t_;
    //the moment when the person moved for the last time
    ros::Time last_movement_;

    //previous target position vector
    tf::Vector3 target_vec_prev_;

    //a deque of 10 last measured velocity values
    std::deque<double> meas_velocities_;
    //counter used to update the deque
    int counter_;
    //mean velocity
    double mean_vel_;

};


#endif // ROBOTCONTROLLER_KINEMATIC_HBZ_TT_H

