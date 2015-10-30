#ifndef ROBOTCONTROLLER_KINEMATIC_SLP_H
#define ROBOTCONTROLLER_KINEMATIC_SLP_H

/// THIRD PARTY
#include <Eigen/Core>
#include <geometry_msgs/PointStamped.h>

/// PROJECT
#include <path_follower/controller/robotcontroller_interpolation.h>
#include <path_follower/utils/parameters.h>
#include <path_follower/pathfollower.h>

class RobotController_Kinematic_SLP : public RobotController_Interpolation
{
public:
    RobotController_Kinematic_SLP(PathFollower *path_driver);
    
    //! Immediately stop any motion.
    virtual void stopMotion();
    
    virtual void start();
    
    
protected:
    virtual MoveCommandStatus computeMoveCommand(MoveCommand* cmd);
    virtual void publishMoveCommand(const MoveCommand &cmd) const;
    
    virtual void initialize();
    
    void laserBack(const sensor_msgs::LaserScanConstPtr& scan_back);
    void laserFront(const sensor_msgs::LaserScanConstPtr& scan_front);
    
private:
    void findMinDistance();
    
private:
    struct ControllerParameters : public RobotController_Interpolation::InterpolationParameters
    {
        P<double> k1;
        P<double> k2;
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
        
        ControllerParameters():
            k1(this, "~k1", 1.0, ""),
            k2(this, "~k2", 1.0, ""),
            gamma(this, "~gamma", 1.0, ""),
            theta_a(this, "~theta_a", M_PI/4.0, ""),
            epsilon(this, "~epsilon", 0.5, ""),
            b(this, "~b", 0.2, ""),
            max_angular_velocity(this, "~max_angular_velocity", 0.8, ""),
            look_ahead_dist(this, "~look_ahead_dist", 0.5, ""),
            k_o(this, "~k_o", 0.3, ""),
            k_g(this, "~k_g", 0.4, ""),
            k_w(this, "~k_w", 0.5, ""),
            k_curv(this, "~k_curv", 0.05, "")
        {}
    } opt_;

    const RobotController_Interpolation::InterpolationParameters& getParameters() const
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
            if ( isnan(speed) || isinf(speed)
                 || isnan(direction_angle) || isinf(direction_angle)
                 || isnan(rotation) || isinf(rotation) )
            {
                ROS_FATAL("Non-numerical values in command: %d,%d,%d,%d,%d,%d",
                          isnan(speed), isinf(speed),
                          isnan(direction_angle), isinf(direction_angle),
                          isnan(rotation), isinf(rotation));
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
    //index of the orthogonal projection to the path
    uint proj_ind_;
    
    //x component of the following error in path coordinates
    double xe_;
    //y component of the following error in path coordinates
    double ye_;
    
    //cumulative curvature sum w.r.t. path
    double curv_sum_;
    //cumulative distance to goal sum w.r.t. path
    double distance_to_goal_;
    //distance to the nearest obstacle
    double distance_to_obstacle_;
};

#endif // ROBOTCONTROLLER_KINEMATIC_SLP_H
