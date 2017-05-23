#ifndef ROBOTCONTROLLER_DYNAMIC_WINDOW
#define ROBOTCONTROLLER_DYNAMIC_WINDOW

/// THIRD PARTY
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>

/// SYSTEM
#include <pcl_ros/point_cloud.h>

/// PROJECT
#include <path_follower/controller/robotcontroller.h>
#include <path_follower/utils/parameters.h>


/// The Dynamic_Window class
class RobotController_Dynamic_Window : public RobotController
{
public:
    /**
     * @brief RobotController_Dynamic_Window
     */
    RobotController_Dynamic_Window();

    //! Immediately stop any motion.
    virtual void stopMotion();

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
     * @brief findNextVelocityPair finds the next velocity pair (v,w) inside the specified dynamic window
     */
    void findNextVelocityPair();
    /**
     * @brief checkAdmissibleVelocities iterates/predicts up to the specified time point, and cheks admissibility
     */
    bool checkAdmissibleVelocities();
    /**
     * @brief setGoalPosition sets the goal position to be the next point on the path in front of the robot
     *
     * In this way, The Dynamic Window Approach, which is actually an obstacle avoidance method, or a local
     * planner, can be used as a path following algorithm, which inherently contains obstacle avoidance.
     *
     */
    void setGoalPosition();
    /**
     * @brief searchMinObstDist searches for nearest obstacle points along the predicted paths
     */
    void searchMinObstDist();


    // nominal robot velocity
    double vn_;
    //time measure for the dwa period
    ros::Time t_old_;
    //velocity commands (newly found velocity pair)
    double v_cmd_, w_cmd_;
    //distance to the nearest obstacle on the curvature (every (v,w) pair uses this variable)
    double curv_dist_obst_;
    //goal position
    double mGoalPosX, mGoalPosY;
    //currently measured (x,y,theta)
    double x_meas_, y_meas_, theta_meas_;
    //orientation of the predicted point
    double theta_pred_;
    //current velocity pair in the dynamic window
    double v_iter_, w_iter_;
    //predicted position
    double x_pred_, y_pred_;
    //next position
    double x_next_, y_next_, theta_next_;
    //obstacle flag
    bool obstacle_found;
    //marker id counter
    int m_id_counter;
    //far predicted positions
    visualization_msgs::MarkerArray far_pred_points;
    //possible trajectories
    nav_msgs::Path traj_;

    //publish the goal position
    ros::Publisher goal_pub;
    //publish the next predicted position
    ros::Publisher predict_pub;
    //publish the line between the predicted position and the closest obstacle
    ros::Publisher obst_marker_pub;
    //publish all far predicted points
    ros::Publisher far_pred_pub;
    //publish the next obstacle point
    ros::Publisher obst_point_pub;
    //publish the possible trajectories
    ros::Publisher traj_pub;


    struct ControllerParameters : public RobotController::ControllerParameters
    {
        P<double> angle_fact;
        P<double> disobst_fact;
        P<double> v_fact;
        P<double> lin_acc;
        P<double> ang_acc;
        P<double> lin_dec;
        P<double> ang_dec;
        P<double> T_dwa;
        P<double> v_step;
        P<double> w_step;
        P<double> fact_T;
        P<double> step_T;
        P<double> obst_dist_thresh;
        P<double> max_ang_vel;
        P<double> initial_vel_fact;

        ControllerParameters():
            RobotController::ControllerParameters("dynamic_window"),

            angle_fact(this, "angle_fact", 0.2, "Factor for regulating the target heading term."),
            disobst_fact(this, "disobst_fact", 0.2, "Factor for regulating the obstacle distance (clearance) term."),
            v_fact(this, "v_fact", 2.0, "Factor for regulating velocity term."),
            lin_acc(this, "lin_acc", 2.0, "Linear acceleration."),
            ang_acc(this, "ang_acc", 0.5, "Angular acceleration,"),
            lin_dec(this, "lin_dec", 0.5, "Linear deceleration."),
            ang_dec(this, "ang_dec", 0.01, "Angular deceleration."),
            T_dwa(this, "T_dwa", 0.125, "Time length defining the dynamic window."),
            v_step(this, "v_step", 0.05, "Linear velocity step of the dynamic window."),
            w_step(this, "w_step", 0.05, "Angular velocity step of the dynamic window."),
            fact_T(this, "fact_T", 5.0, "Factor for multiplying the dynamic window time length."),
            step_T(this, "step_T", 0.4, "Time step inside the dynamic window time lenght"),
            obst_dist_thresh(this, "obst_dist_thresh", 0.6, "Threshold at which the obstacles are taken into account."),
            max_ang_vel(this, "max_ang_vel", 0.5, "Maximum angular velocity."),
            initial_vel_fact(this, "initial_vel_fact", 0.2, "Factor for scaling the initial velocity commands.")
        {}
    } opt_;

    const RobotController::ControllerParameters& getParameters() const
    {
        return opt_;
    }

    struct Command
    {
        RobotController_Dynamic_Window *parent_;

        //! Speed of the movement.
        float speed;
        //! Direction of movement as angle to the current robot orientation.
        float direction_angle;
        //! rotational velocity.
        float rotation;


        // initialize all values to zero
        Command(RobotController_Dynamic_Window *parent):
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

#endif // ROBOTCONTROLLER_DYNAMIC_WINDOW

