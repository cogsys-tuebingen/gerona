#ifndef ROBOTCONTROLLER_VELOCITY_TT_H
#define ROBOTCONTROLLER_VELOCITY_TT_H

/// THIRD PARTY
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <pcl_ros/point_cloud.h>

/// PROJECT
#include <path_follower/controller/robotcontroller.h>
#include <path_follower/utils/parameters.h>


/// SYSTEM
#include <nav_msgs/Odometry.h>


class RobotController_Velocity_TT: public RobotController
{
public:
    /**
     * @brief RobotController_Kinematic_HBZ
     */
    RobotController_Velocity_TT();

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
        P<double> dist_thresh;
        P<double> krep;
        P<double> katt;
        P<int> input_type;
        P<bool> use_odom_twist;
        P<int> number_target_poses;
        P<double> robot_radius;
        P<double> dest_distance;
        P<double> min_scale_fact;
        P<double> dist_scale_fact;
        P<double> max_angular_velocity;
        P<double> kangular;



        ControllerParameters(const std::string& name = "velocity_TT"):

            RobotController::ControllerParameters(name),
            dist_thresh(this, "dist_thresh", 0.5, "dist_thresh description."),
            krep(this, "krep", 0.1, "repulsive force wight"),
            katt(this, "katt", 0.2, "attractive force weight"),
            input_type(this, "input_type", 0, "input type: 0 = odom, 1 = Path, 2 = TF "),
            use_odom_twist(this, "use_odom_twist", false, "set to true if odom.twist should be used, otherwise velocity is estimated."),
            number_target_poses(this, "number_target_poses", 3, "number of target poses used for velocity estimate "),
            robot_radius(this, "robot_radius", 0.25, "radius of enclosing circle of the robot"),
            dest_distance(this, "dest_distance", 0.3, "desired distance to target"),
            min_scale_fact(this, "min_scale_fact", 0.1, "min velocity adjustment factor"),
            dist_scale_fact(this, "dist_scale_fact", 100.0, "factor by which velociy adjustment increases by distance"),
            max_angular_velocity(this, "max_angular_velocity", 0.8, "max_angular_velocity"),
            kangular(this, "kangular", 3.0, "angular_velocity adjustment rate")

        {}
    } opt_;

    const RobotController::ControllerParameters& getParameters() const
    {
        return opt_;
    }

    /*
    struct Command
    {
        RobotController_Velocity_TT *parent_;

        //! Speed of the movement.
        float vx;
        //! Direction of movement as angle to the current robot orientation.
        float vy;
        //! rotational velocity.
        float rotation;


        // initialize all values to zero
        Command(RobotController_Velocity_TT *parent):
            parent_(parent),
            vx(0.0f), vy(0.0f), rotation(0.0f)
        {}

        operator MoveCommand()
        {
            Eigen::Vector2f dir(vx,vy);
            dir[0] = vx;
            dir[1] = vy;
            MoveCommand mcmd(true);
            mcmd.setDirection(dir);
            mcmd.setVelocity(sqrt(vx*vx+vy*vy));
            mcmd.setRotationalVelocity(rotation);
            return mcmd;
        }

        bool isValid()
        {
            if ( std::isnan(vx) || std::isinf(vx)
                 || std::isnan(vy) || std::isinf(vy)
                 || std::isnan(rotation) || std::isinf(rotation) )
            {
                ROS_FATAL("Non-numerical values in command: %d,%d,%d,%d,%d,%d",
                          std::isnan(vx), std::isinf(vx),
                          std::isnan(vy), std::isinf(vy),
                          std::isnan(rotation), std::isinf(rotation));
                // fix this instantly, to avoid further problems.
                vx = 0.0;
                vy = 0.0;
                rotation = 0.0;

                return false;
            } else {
                return true;
            }
        }
    };

    Command cmd_;
    */


    void reset();
    void setPath(Path::Ptr path);

    void odomCallback (const nav_msgs::OdometryConstPtr& odom);

    void processPose(const geometry_msgs::PoseStamped &odom);


    cv::Vec2f estimateVel();

    cv::Vec2f CalcForceRep(bool &hasObst);
    cv::Vec2f CalcForceRep(const pcl::PointCloud<pcl::PointXYZ>& cloud, bool &hasObst);
    cv::Vec2f CalcForceAtt();

    enum INPUT_TYPE{ VELTT_IT_ODOM = 0, VELTT_IT_PATH, VELTT_IT_TF};

    geometry_msgs::PoseStamped Odom2Pose(nav_msgs::Odometry dom);

    bool  poseToGlobalPose(geometry_msgs::PoseStamped in_pose, geometry_msgs::PoseStamped &global);
    bool  poseToLocalPose(geometry_msgs::PoseStamped in_pose, geometry_msgs::PoseStamped &local);



private:

    std::deque<geometry_msgs::PoseStamped> targetPoses_;
    ros::Subscriber subTargetOdom_;

    cv::Vec2f lastTargetDir_;
    //cv::Vec2f lastTargetPos_;



};

#endif // ROBOTCONTROLLER_KINEMATIC_HBZ_H
