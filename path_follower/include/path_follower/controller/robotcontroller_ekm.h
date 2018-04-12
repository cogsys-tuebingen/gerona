#ifndef ROBOTCONTROLLER_EKM_H
#define ROBOTCONTROLLER_EKM_H

/// THIRD PARTY
#include <Eigen/Core>

/// PROJECT
#include <path_follower/controller/robotcontroller.h>
#include <path_follower/utils/parameters.h>


class RobotController_EKM: public RobotController
{
public:
    /**
     * @brief RobotController_Kinematic_HBZ
     */
    RobotController_EKM();

    /**
     * @brief stopMotion stops the robot
     */
    virtual void stopMotion();
    /**
     * @brief start
     */
    virtual void start();


protected:
    /**
     * @brief computeMoveCommand computes the command velocity for the robot
     *
     * The command velocity is computed for each controller differently. This is the core of
     * every controller. For more details, please visit: https://github.com/cogsys-tuebingen/gerona/wiki/controllers
     * On this wiki page, you will find references for each controller, where more mathematical and experimental details
     * can be found.
     *
     * @param cmd
     */
    virtual MoveCommandStatus computeMoveCommand(MoveCommand* cmd);
    /**
     * @brief publishMoveCommand publishes the computed move command
     *
     * The command velocity is set depending on the kinematics of the robot. E.g. for
     * differential drives the command input is (v, w), where v is linear, and w angular velocity,
     * and for an Ackermann drive, the command input is (v, phi), where v is linear velocity, and
     * phi is the steering angle. For an omnidirectional vehicle, it is possible to directly set
     * the linear velocity and the direction angle, while the rotation is set independently.
     *
     * @param cmd
     */
    virtual void publishMoveCommand(const MoveCommand &cmd) const;
    /**
     * @brief initialize
     */
    virtual void initialize();
    /**
     * @brief WheelVelocities computes the speed of the left and right wheels
     *
     * Currently not used, but could be helpful.
     *
     */
    //void WheelVelocities(const std_msgs::Float64MultiArray::ConstPtr& array);
    /**
     * @brief computeSpeed computes the actual command velocity, by using different speed control techniques
     */
    //virtual double computeSpeed();

private:
    void findMinDistance();

protected:
    struct ControllerParameters : public RobotController::ControllerParameters
    {
        P<int> n;
        P<double> df;
        P<double> ts;
        P<double> a;
        P<double> alpha;
        P<double> u_d;

        P<double> kx;
        P<double> ky;
        P<double> kw;
        P<double> lx;
        P<double> ly;
        P<double> lw;

        ControllerParameters(const std::string& name = "ekm"):

            RobotController::ControllerParameters(name),
            n(this, "n", 1, "kx description."),
            df(this, "df", 0.1, "min distance to goal."),
            ts(this, "ts", 0.1, "timestep."),
            a(this, "a", 0.8, "a description."),
            alpha(this, "alpha", 0.0, "ly description."),
            u_d(this, "u_d", 0.3, "lz description."),

            kx(this, "kx", 1.0, "kx description."),
            ky(this, "ky", 1.0, "ky description."),
            kw(this, "kw", 5.0, "kz description."),
            lx(this, "lx", 0.4, "lx description."),
            ly(this, "ly", 0.4, "ly description."),
            lw(this, "lw", 0.75, "lz description.")

        {}
    } opt_;

    const RobotController::ControllerParameters& getParameters() const
    {
        return opt_;
    }

    struct Command
    {
        RobotController_EKM *parent_;

        //! Speed of the movement.
        float speed;
        //! Direction of movement as angle to the current robot orientation.
        float direction_angle;
        //! rotational velocity.
        float rotation;


        // initialize all values to zero
        Command(RobotController_EKM *parent):
            parent_(parent),
            speed(0.0f), direction_angle(0.0f), rotation(0.0f)
        {}

        operator MoveCommand()
        {
            MoveCommand mcmd(true);
            mcmd.setDirection(direction_angle);
            mcmd.setVelocity(speed);
            mcmd.setRotationalVelocity(rotation);
            return mcmd;
        }

        bool isValid()
        {
            if ( std::isnan(speed) || std::isinf(speed)
                 || std::isnan(direction_angle) || std::isinf(direction_angle)
                 || std::isnan(rotation) || std::isinf(rotation) )
            {
                ROS_FATAL("Non-numerical values in command: %d,%d,%d,%d,%d,%d",
                          std::isnan(speed), std::isinf(speed),
                          std::isnan(direction_angle), std::isinf(direction_angle),
                          std::isnan(rotation), std::isinf(rotation));
                // fix this instantly, to avoid further problems.
                speed = 0.0;
                direction_angle = 0.0;
                rotation = 0.0;

                return false;
            } else {
                return true;
            }
        }
    };

    Command cmd_;


    void reset();
    void setPath(Path::Ptr path);

    void DerivePathInterp(double fact);


private:
    //std::vector<double> x_p_;
    //std::vector<double> y_p_;
    std::vector<double> phi_p_;
    double prev_phi_d_;
    bool has_prev_phi_d_;
    //double phi_;



};

#endif // ROBOTCONTROLLER_KINEMATIC_HBZ_H
