/*
 * BehaviouralPathDriver.h
 *
 *  Created on: Apr 15, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef BEHAVIOURALPATHDRIVER_H
#define BEHAVIOURALPATHDRIVER_H

/// PROJECT
#include "MotionController.h"
#include "PidCtrl.h"

/// SYSTEM
#include <ros/ros.h>
#include <nav_msgs/Path.h>
//#include <ramaxx_msgs/RamaxxMsg.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>

namespace motion_control {

class BehaviouralPathDriver : public MotionController
{
public:
    friend class Behaviour;

    struct Waypoint
    {
        Waypoint() {}
        Waypoint(const geometry_msgs::PoseStamped& ref)
        {
            x = ref.pose.position.x;
            y = ref.pose.position.y;
            theta = tf::getYaw(ref.pose.orientation);
        }
        operator geometry_msgs::Pose() const
        {
            geometry_msgs::Pose result;
            result.position.x = x;
            result.position.y = y;
            result.orientation = tf::createQuaternionMsgFromYaw(theta);
            return result;
        }

        double x;
        double y;
        double theta;
    };

    typedef std::vector<Waypoint> Path;

    struct Command
    {
        double velocity;
        double steer_front;
        double steer_back;

        /* ramaxx_msg commented for the moment, as it would limit the use of this node to the rabots.
         * Reimplement this, if you want to use a robot with front and back steerting.
         */
        /*
        operator ramaxx_msgs::RamaxxMsg()
        {
            ramaxx_msgs::RamaxxMsg msg;
            msg.data.resize(3);
            msg.data[0].key = ramaxx_msgs::RamaxxMsg::CMD_STEER_FRONT_DEG;
            msg.data[1].key = ramaxx_msgs::RamaxxMsg::CMD_STEER_REAR_DEG;
            msg.data[2].key = ramaxx_msgs::RamaxxMsg::CMD_SPEED;
            msg.data[0].value = steer_front * 180.0/M_PI;
            msg.data[1].value = steer_back * 180.0/M_PI;
            msg.data[2].value = v;
            return msg;
        }
        */

        operator geometry_msgs::Twist()
        {
            geometry_msgs::Twist msg;
            msg.linear.x  = velocity;
            msg.angular.z = steer_front;
            return msg;
        }
    };

    struct Options
    {
        Options()
        { reset(); }

        void reset() {
            path_idx = 0;
            wp_idx = 0;
        }

        double wp_tolerance_;
        double goal_tolerance_;
        double l_;
        double dead_time_;

        //! Minimum speed of the robot (needed, as the outdoor buggys can't handle velocities below about 0.3).
        float min_velocity_;
        //! Maximum velocity (to prevent the high level control from running amok).
        float max_velocity_;
        //! Desired velocity (defined by the action goal).
        float velocity_;

        double steer_slow_threshold_;

        //! Maximum distance the robot is allowed to depart from the path. If this threshold is exceeded,
        //! the path follower will abort.
        double max_distance_to_path_;

        //! Width of the collisin box for obstacle avoidance.
        float collision_box_width_;
        //! Minimum width of the collision box for obstacle avoidance (grows with increasing velocity).
        float collision_box_min_length_;
        //! This factor determines, how much the length of the box is increased, depending on the velocity.
        float collision_box_velocity_factor_;

        int path_idx;
        int wp_idx;
    };

    struct NullBehaviour {
    };

    class Behaviour
    {
    protected:
        Behaviour(BehaviouralPathDriver& parent)
            : parent_(parent)
        {}
        Path& getSubPath(unsigned index);
        int getSubPathCount() const;
        PathFollower& getNode();

        PidCtrl& getPid();
        Command& getCommand();
        Options& getOptions();
        double distanceTo(const Waypoint& wp);

    public:
        virtual ~Behaviour() {}

        virtual void execute(int* status) = 0;

    protected:
        BehaviouralPathDriver& parent_;
    };


public:
    BehaviouralPathDriver(ros::Publisher& cmd_pub, PathFollower *node);

    virtual void start();
    virtual void stop();
    virtual int getType();

    virtual int execute(path_msgs::FollowPathFeedback& fb, path_msgs::FollowPathResult& result);
    virtual void configure();
    virtual void setGoal(const path_msgs::FollowPathGoal& goal);

    void publishCommand();
    PathFollower* getNode() const;

    void setPath(const nav_msgs::Path& path);
    void drawArrow(int id, const geometry_msgs::Pose &pose, const std::string& ns, float r, float g, float b, double live = 3);
    void drawLine(int id, const geometry_msgs::Point &from, const geometry_msgs::Point &to, const string &frame, const std::string& ns, float r, float g, float b, double live = 3, float scale = 0.3);
    void drawCircle(int id, const geometry_msgs::Point &center, double radius, const string &frame, const std::string& ns, float r, float g, float b, double live = 3);
    void drawMark(int id, const geometry_msgs::Point &pos, const std::string& ns, float r, float g, float b);
    void predictPose(Vector2d &front_pred,
                      Vector2d &rear_pred );

    Vector3d getSlamPose()
    {
        return slam_pose_;
    }

    bool simpleCheckCollision(float box_width, float box_length, int dir_sign);

    bool checkCollision();

protected:
    void clearActive();

private:
    PathFollower* node_;

    ros::NodeHandle private_nh_;

    ros::Publisher& cmd_pub_;
    ros::Publisher vis_pub_;

    ros::Subscriber laser_sub_;

    Behaviour* active_behaviour_;

    Command current_command_;
    Options options_;

    Vector3d slam_pose_;
    nav_msgs::Path path_;
    std::vector<std::vector<Waypoint> > paths_;

    PidCtrl pid_;

    int pending_error_;
};

}

#endif // BEHAVIOURALPATHDRIVER_H
