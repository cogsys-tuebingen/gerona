#ifndef PATHFOLLOWER_H
#define PATHFOLLOWER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <nav_msgs/Odometry.h>
#include <path_msgs/FollowPathAction.h>
#include <tf/transform_listener.h>

//#include <utils/LibUtil/LowPassFilter.h> //FIXME: wurde der noch bebraucht?

#include "BehaviouralPathDriver.h"
#include "stucktimeout.h"

class PathFollower
{
public:
    PathFollower(ros::NodeHandle &nh);
    ~PathFollower();

    bool getWorldPose(Vector3d& pose, geometry_msgs::Pose* pose_out = NULL) const;
    bool transformToLocal(const geometry_msgs::PoseStamped& global, geometry_msgs::PoseStamped& local );
    bool transformToLocal(const geometry_msgs::PoseStamped& global, Vector3d& local );
    bool transformToGlobal(const geometry_msgs::PoseStamped& local, geometry_msgs::PoseStamped& global );

    void update();

private:
    typedef actionlib::SimpleActionServer<path_msgs::FollowPathAction> FollowPathServer;

    struct Options
    {
        //! If the robot is not moving for at least the defined number of seconds, the path execution is aborted.
        float stuck_timeout;

        //! Tolerance in the robot movement. If the robot moves less than this value in one stuck-timer period, it is
        //! assumed to be stuck.
        float stuck_pos_tolerance;
    };


    ros::NodeHandle node_handle_;

    Options opt_;

    //! Action server that communicates with path_control (or who ever sends actions)
    FollowPathServer follow_path_server_;

    //! TODO: was macht der?
    //LowPassFilter<float> speed_filter_;

    //! Publisher for driving commands.
    ros::Publisher cmd_pub_;
    //! Subscriber for odometry messages.
    ros::Subscriber odom_sub_;
    MotionController *active_ctrl_;

    //! The last received odometry message.
    nav_msgs::Odometry odometry_;

    //! Name of the world frame (default: /map)
    std::string world_frame_;
    //! Name of the robot frame (default: /base_link)
    std::string robot_frame_;

    tf::TransformListener pose_listener_;

    //! Timeout to detect if the robot is stuck (= does not move while following the path).
    StuckTimeout *stuck_timeout_;


    void followPathGoalCB();
    void followPathPreemptCB();

    void odometryCB(const nav_msgs::OdometryConstPtr &odom);
};

#endif // PATHFOLLOWER_H
