#ifndef ROBOTCONTROLLER_POTENTIAL_FIELD_H
#define ROBOTCONTROLLER_POTENTIAL_FIELD_H

/// THIRD PARTY
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>

/// SYSTEM
#include <pcl_ros/point_cloud.h>

/// PROJECT
#include <path_follower/controller/robotcontroller.h>
#include <path_follower/utils/parameters.h>


/// The Potential_Field class
class RobotController_Potential_Field: public RobotController
{
public:
    /**
     * @brief RobotController_Potential_Field
     */
    RobotController_Potential_Field();

    //! Immediately stop any motion.
    virtual void stopMotion();

    /// Sets the speed of the robot.
    static void setRobotSpeed(double speed);

protected:

    /// Sets the goal position.
    virtual void setGoalPosition();

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
     * @brief initializeMarkers initializes the markers representing attractive and repulsive forces
     */
    void initializeMarkers();
    /**
     * @brief visualizeMarkers visualizes the markers representing attractive and repulsive forces
     */
    void visualizeMarkers();

    // x and y component of the attractive force vector
    double FAttX, FAttY;
    // x and y component of the resulting force vector
    double FResX, FResY;
    // nominal robot velocity
    double vn_;
    // error coordinates
    double xe_, ye_, theta_e_;

    // nearest obstacle for each segment, split in x and y component
    double obstacles[2];
    // repulsive force for each segment, split in x and y component
    Eigen::Vector2d FRep;

    // update the current potential field
    void update(double newFAttX, double newFAttY);
    // dind the nearest obstacle for each segment
    void findObstacles();
    // compute the repulsive forces
    void computeFReps();
    // compute the resulting force acting on the robot
    void computeFRes();

    double mGoalPosX;
    double mGoalPosY;

    // some markers for visualizing the force vectors
    // for FAtt
    visualization_msgs::Marker marker;
    // for FReps
    visualization_msgs::Marker FrepMarker;
    // for FRes
    visualization_msgs::Marker FResMarker;
    // MarkerArray for publishing
    visualization_msgs::MarkerArray FMarkers;
    // marker publisher
    ros::Publisher F_pub;



    struct ControllerParameters : public RobotController::ControllerParameters
    {
        P<double> kAtt;
        P<double> kRep;
        P<double> dist_thresh;
        P<double> max_angular_velocity;

        ControllerParameters():
            RobotController::ControllerParameters("potential_field"),

            kAtt(this, "kAtt", 0.2, "Factor for the attractive force influence."),
            kRep(this, "kRep", 0.5, "Factor for the repulsive force influence."),
            dist_thresh(this, "dist_thres", 2.5, "Distance at which the obstacles are taken into account."),
            max_angular_velocity(this, "max_angular_velocity", 0.8, "Maximum angular velocity.")
        {}
    } opt_;

    const RobotController::ControllerParameters& getParameters() const
    {
        return opt_;
    }

    struct Command
    {
        RobotController_Potential_Field *parent_;

        //! Speed of the movement.
        float speed;
        //! Direction of movement as angle to the current robot orientation.
        float direction_angle;
        //! rotational velocity.
        float rotation;


        // initialize all values to zero
        Command(RobotController_Potential_Field *parent):
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


};
#endif //ROBOTCONTROLLER_POTENTIAL_FIELD_H
