#ifndef ROBOTCONTROLLER_DYNAMIC_WINDOW
#define ROBOTCONTROLLER_DYNAMIC_WINDOW

/// THIRD PARTY
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>

/// SYSTEM
#include <pcl_ros/point_cloud.h>
#include <Eigen/Core>
#include <Eigen/Dense>


/// PROJECT
#include <path_follower/controller/robotcontroller_interpolation.h>
#include <path_follower/utils/parameters.h>


/// The Dynamic_Window class
class RobotController_Dynamic_Window : public RobotController_Interpolation
{
public:
    RobotController_Dynamic_Window();

    //! Immediately stop any motion.
    virtual void stopMotion();

protected:

    virtual MoveCommandStatus computeMoveCommand(MoveCommand* cmd);
    virtual void publishMoveCommand(const MoveCommand &cmd) const;
    virtual void initialize();

    void calculateMovingDirection();
    void findNextVelocityPair();
    bool checkAdmissibleVelocities();
    void setGoalPosition();
    void searchMinObstDist();


    // nominal robot velocity
    double vn_;
    // index of the orthogonal projection
    unsigned int proj_ind_;
    // error coordinates
    double xe_, ye_, theta_e_;
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


    struct ControllerParameters : public RobotController_Interpolation::InterpolationParameters
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
            angle_fact(this, "~angle_fact", 0.2, ""),
            disobst_fact(this, "~disobst_fact", 0.2, ""),
            v_fact(this, "~v_fact", 2.0, ""),
            lin_acc(this, "~lin_acc", 2.0, ""),
            ang_acc(this, "~ang_acc", 0.5, ""),
            lin_dec(this, "~lin_dec", 0.5, ""),
            ang_dec(this, "~ang_dec", 0.01, ""),
            T_dwa(this, "~T_dwa", 0.125, ""),
            v_step(this, "~v_step", 0.05, ""),
            w_step(this, "~w_step", 0.05, ""),
            fact_T(this, "~fact_T", 5.0, ""),
            step_T(this, "~step_T", 0.4, ""),
            obst_dist_thresh(this, "~obst_dist_thresh", 0.6, ""),
            max_ang_vel(this, "~max_ang_vel", 0.5, ""),
            initial_vel_fact(this, "~initial_vel_fact", 0.2, "")
        {}
    } opt_;

    const RobotController_Interpolation::InterpolationParameters& getParameters() const
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

