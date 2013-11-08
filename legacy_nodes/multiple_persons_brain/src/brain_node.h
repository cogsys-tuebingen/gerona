/**
 * @file brain_node.h
 * @date July 2012
 * @author marks
 */

#ifndef _BRAIN_NODE_H_
#define _BRAIN_NODE_H_

// C/C++
#include <vector>
#include <string>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>

namespace multiple_persons_brain {

/**
 * @brief Represents one simulated person.
 */
struct Person {

    /// Current target
    geometry_msgs::Point target;

    /// Index in stage
    int idx;

    /// Flag is we have send a target to this person
    bool has_target;

    /// Used to publish new targets
    ros::Publisher target_pub;

    /// Current pose
    geometry_msgs::Pose pose;

    /// Used to determine if the robot is moving or not
    int stalled_count;

    /**
     * @brief Create a person.
     */
    Person()
        : idx( 0 ), has_target( false ), stalled_count( 0 )
    {}
};

class BrainNode {
public:

    /**
     * @brief Read config an initialize object.
     */
    BrainNode();

    /**
     * @brief Run the node and process messages etc.
     */
    void spin();

    /**
     * @brief Check persons position and set new targets
     *      if neccessary.
     */
    void updateCallback( const ros::TimerEvent& e );

protected:

    void selectAndPublishTarget( Person& person, const geometry_msgs::Pose& p );

    /**
     * @brief Get the current position of a simulated person.
     *
     * @param robot_idx Index of that person
     * @param p Current pose
     * @return False if an error occurred. True if the pose is valid.
     */
    bool getRobotPose( const int robot_idx, geometry_msgs::Pose &p );

    std::string getRobotName( const int robot_idx, const std::string &name );

    /// Node handle
    ros::NodeHandle n_;

    /// Used to get the positions of the robots
    tf::TransformListener tf_;

    /// Update timer
    ros::Timer update_timer_;

    /// List of all target points
    std::vector<geometry_msgs::Point> targets_;

    /// All simulated persons
    std::vector<Person> persons_;

    /// Name of the world frame
    std::string map_frame_;

    /// Startup delay in seconds
    double start_delay_;
};

} // namespace

#endif // _BRAIN_NODE_H_
