#ifndef ROBOTCONTROLLER_MODELBASED_H
#define ROBOTCONTROLLER_MODELBASED_H

/// THIRD PARTY
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>

/// PROJECT
#include <path_follower/controller/robotcontroller.h>
#include <path_follower/utils/parameters.h>
#include <path_follower/parameters/model_parameters.h>


/// LIBRARIES
#include <model_based_planner/imodelbasedplanner.h>


#define MODEL_CONTROLLER_DEBUG 1

#ifdef MODEL_CONTROLLER_DEBUG
    #include "model_based_planner/pose_writer.h"
#endif


/// The RobotController_ModelBased
class RobotController_ModelBased: public RobotController
{
public:
    /**
     * @brief RobotController_Model_Based
     */
    RobotController_ModelBased();

    //! Immediately stop any motion.
    virtual void stopMotion();

    /// Sets the speed of the robot.
    static void setRobotSpeed(double speed);

protected:

    /// Sets the goal position.
    //virtual void setGoalPosition();

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

    bool CheckNextPath();


    struct ControllerParameters : public RobotController::ControllerParameters
    {
        // Command Execution
        P<bool> use_lin_velocity;
        P<bool> use_ang_velocity;
        P<int> min_traj_nodes, min_traj_nodes_goal;
        P<double> threshold_velocity;

        P<std::string> pose_output_folder;


        ControllerParameters():
            RobotController::ControllerParameters("modelbased"),
            // Command execution
            use_lin_velocity(this, "use_lin_velocity", false, "Determines if the current linear velocity is used by the local planner"),
            use_ang_velocity(this, "use_ang_velocity", false, "Determines if the current angular velocity is used by the local planner"),
            min_traj_nodes(this, "min_traj_nodes", 15, "Minimum number of poses in result trajectory"),
            min_traj_nodes_goal(this, "min_traj_nodes_goal", 15, "Minimum number of poses in result trajectory when goal is reachable"),
            threshold_velocity(this, "threshold_velocity", 1.0, "Lower velocity bound for model based path search"),
            pose_output_folder(this, "pose_output_folder", "", "Output folder for debug output")

        {

        }
    } opt_;

    struct MControllerParameters : public ModelParameters
    {
        MControllerParameters():
            ModelParameters()
        {

        }
    } m_opt_;

    
    struct Command
    {
        RobotController_ModelBased *parent_;

        //! Speed of the movement.
        float speed;
        //! Direction of movement as angle to the current robot orientation.
        float direction_angle;
        //! rotational velocity.
        float rotation;


        // initialize all values to zero
        Command(RobotController_ModelBased *parent):
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
    Waypoint target_;
    Waypoint goal_;
    std::vector<cv::Point3f> currentPath_;
    tf::Transformer* transformer_;
    int commandStatus;
    bool doPlan_;
    ModelBasedPlannerConfig config;
    std::string localMapFrame_;



    enum MBC_CommandStatus
    {
        OKAY = 0, REACHED_GOAL, COLLISON, ERROR
    };

    bool targetTransform2base(ros::Time& now);



    const RobotController::ControllerParameters& getParameters() const
    {
        return opt_;
    }


    void start();

    void reset();
    //void setPath(Path::Ptr path);



    void imageCallback (const sensor_msgs::ImageConstPtr& image);

    bool GetTransform(ros::Time time,std::string targetFrame, std::string sourceFrame, tf::StampedTransform &trans);

    void TransformPath(tf::Transform trans);



    bool initialized_;

    double totalPlanningTime_;
    double maxPlanningTime_;
    int frameCounter_;

    ros::Subscriber subDEM_;

    IModelBasedPlanner::Ptr model_based_planner_;

#ifdef MODEL_CONTROLLER_DEBUG
    ros::Publisher dbgImgPub_;
    PoseWriter writer_;
#endif



};



#endif // ROBOTCONTROLLER_MODELBASED_H
