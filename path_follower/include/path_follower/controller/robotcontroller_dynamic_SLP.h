#ifndef ROBOTCONTROLLER_DYNAMIC_SLP_H
#define ROBOTCONTROLLER_DYNAMIC_SLP_H

/// THIRD PARTY
#include <Eigen/Core>
#include <geometry_msgs/PointStamped.h>

/// PROJECT
#include <path_follower/controller/robotcontroller.h>
#include <path_follower/utils/parameters.h>


class RobotController_Dynamic_SLP: public RobotController
{
public:
    RobotController_Dynamic_SLP();

    //! Immediately stop any motion.
    virtual void stopMotion();

    virtual void start();


protected:
    virtual MoveCommandStatus computeMoveCommand(MoveCommand* cmd);
    virtual void publishMoveCommand(const MoveCommand &cmd) const;

    virtual void initialize();

private:
    void findMinDistance();

private:
    struct ControllerParameters : public RobotController::InterpolationParameters
    {
        P<double> k1;
        P<double> k2;
        P<double> k3;
        P<double> k4;
        P<double> gamma;
        P<double> theta_a;
        P<double> epsilon;
        P<double> b;
        P<double> max_angular_velocity;
        P<double> look_ahead_dist;
        P<double> k_o;
        P<double> k_g;
        P<double> k_w;
        P<double> k_curv;
        P<double> m;
        P<double> I;
        P<double> r;
        P<double> w;
        P<double> gearbox;
        P<double> Kt;
        P<double> max_current;

        ControllerParameters():
            k1(this, "~k1", 1.0, ""),
            k2(this, "~k2", 1.0, ""),
            k3(this, "~k3", 1.0, ""),
            k4(this, "~k4", 1.0, ""),
            gamma(this, "~gamma", 1.0, ""),
            theta_a(this, "~theta_a", M_PI/4.0, ""),
            epsilon(this, "~epsilon", 0.5, ""),
            b(this, "~b", 0.2, ""),
            max_angular_velocity(this, "~max_angular_velocity", 0.8, ""),
            look_ahead_dist(this, "~look_ahead_dist", 0.5, ""),
            k_o(this, "~k_o", 0.3, ""),
            k_g(this, "~k_g", 0.4, ""),
            k_w(this, "~k_w", 0.5, ""),
            k_curv(this, "~k_curv", 0.05, ""),
            m(this, "~m", 50.0, ""),
            I(this, "~I", 0.0356, ""),
            r(this, "~r", 0.10, ""),
            w(this, "~w", 0.2335, ""),
            gearbox(this, "~gearbox", 12.52, ""),
            Kt(this, "~Kt", 0.08, ""),
            max_current(this, "~max_current", 12.0, "")
        {}
    } opt_;

    const RobotController::InterpolationParameters& getParameters() const
    {
        return opt_;
    }

    struct Command
    {
        RobotController_Dynamic_SLP *parent_;

        float tau_fl;
        float tau_fr;
        float tau_br;
        float tau_bl;


        // initialize all values to zero
        Command(RobotController_Dynamic_SLP *parent):
            parent_(parent),
            tau_fl(0.0f), tau_fr(0.0f), tau_br(0.0f), tau_bl(0.0)
        {}

        operator MoveCommand()
        {
            MoveCommand mcmd(true, true);
            mcmd.setWheelTorques(tau_fl, tau_fr, tau_br, tau_bl);
            return mcmd;
        }

        bool isValid()
        {
            if ( std::isnan(tau_fl) || std::isinf(tau_fl)
                 || std::isnan(tau_fr) || std::isinf(tau_fr)
                 || std::isnan(tau_br) || std::isinf(tau_br)
                 || std::isnan(tau_bl) || std::isinf(tau_bl))
            {
                ROS_FATAL("Non-numerical values in command: %d,%d,%d,%d,%d,%d,%d,%d",
                          std::isnan(tau_fl), std::isinf(tau_fl),
                          std::isnan(tau_fr), std::isinf(tau_fr),
                          std::isnan(tau_br), std::isinf(tau_br),
                          std::isnan(tau_bl), std::isinf(tau_bl));
                // fix this instantly, to avoid further problems.
                tau_fl = 0.0;
                tau_fr = 0.0;
                tau_br = 0.0;
                tau_bl = 0.0;

                return false;
            } else {
                return true;
            }
        }
    };

    Command cmd_;

    ros::Subscriber look_at_sub_;
    ros::Subscriber look_at_cmd_sub_;

    ros::Subscriber laser_sub_front_;
    ros::Subscriber laser_sub_back_;

    std::vector<float> ranges_front_;
    std::vector<float> ranges_back_;

    void reset();
    void setPath(Path::Ptr path);

    //nominal velocity
    double vn_;
    //sampling time
    double Ts_;

    //index of the current point on the path (origin of the F-S frame)
    uint ind_;
    //index of the orthogonal projection to the path
    uint proj_ind_;

    //x component of the following error in path coordinates
    double xe_;
    //y component of the following error in path coordinates
    double ye_;
    //orientation in path coordinates
    double theta_e_;
    //function for transient maneuvers
    double delta_;
    //moving point dynamics
    double s_prim_;

    //longitudinal speed
    double vx_;
    //desired rotation control
    double zeta_;
    //rotation control error
    double epsilon_;


    //cumulative curvature sum w.r.t. path
    double curv_sum_;
    //cumulative distance to goal sum w.r.t. path
    double distance_to_goal_;
    //distance to the nearest obstacle
    double distance_to_obstacle_;

    //maximal torque
    double max_torque_;
};

#endif // ROBOTCONTROLLER_DYNAMIC_SLP_H
