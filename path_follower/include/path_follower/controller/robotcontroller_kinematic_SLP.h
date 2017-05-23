#ifndef ROBOTCONTROLLER_KINEMATIC_SLP_H
#define ROBOTCONTROLLER_KINEMATIC_SLP_H

/// THIRD PARTY
#include <geometry_msgs/PointStamped.h>

/// PROJECT
#include <path_follower/controller/robotcontroller.h>
#include <path_follower/utils/parameters.h>


class RobotController_Kinematic_SLP: public RobotController
{
public:
    /**
     * @brief RobotController_Kinematic_SLP
     */
    RobotController_Kinematic_SLP();
    
    //! Immediately stop any motion.
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
    
private:
    void findMinDistance();
    
private:
    struct ControllerParameters : public RobotController::ControllerParameters
    {
        P<double> k1;
        P<double> k2;
        P<double> gamma;
        P<double> theta_a;
        P<double> epsilon;
        P<double> b;
        P<double> max_angular_velocity;
        
        ControllerParameters():
            RobotController::ControllerParameters("kinematic_SLP"),

            k1(this, "k1", 1.0, "Factor for tuning the speed of the virtual vehicle."),
            k2(this, "k2", 1.0, "Factor for tuning the angular velocity command."),
            gamma(this, "gamma", 1.0, "Factor for tuning the angular velocity command."),
            theta_a(this, "theta_a", M_PI/4.0, "Factor for tuning the transient maneuvers."),
            epsilon(this, "epsilon", 0.5, "Error threshold for the speed control."),
            b(this, "b", 0.2, "Factor for tuning the speed control."),
            max_angular_velocity(this, "max_angular_velocity", 0.8, "Maximum angular velocity")
        {}
    } opt_;

    const RobotController::ControllerParameters& getParameters() const
    {
        return opt_;
    }
    
    struct Command
    {
        RobotController_Kinematic_SLP *parent_;
        
        //! Speed of the movement.
        float speed;
        //! Direction of movement as angle to the current robot orientation.
        float direction_angle;
        //! rotational velocity.
        float rotation;
        
        
        // initialize all values to zero
        Command(RobotController_Kinematic_SLP *parent):
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
    //function for transient maneuvers
    double delta_;
    //sampling time
    double Ts_;

    //index of the current point on the path (origin of the F-S frame)
    uint ind_;
    
    //x component of the following error in path coordinates
    double xe_;
    //y component of the following error in path coordinates
    double ye_;
};

#endif // ROBOTCONTROLLER_KINEMATIC_SLP_H
