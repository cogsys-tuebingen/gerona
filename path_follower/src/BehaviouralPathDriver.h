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
#include "vector_field_histogram.h"
#include "visualizer.h"
#include "path.h"
#include "robotcontroller.h"

/// SYSTEM
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>

class BehaviouralPathDriver : public MotionController
{
public:
    friend class Behaviour;

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

        //! Minimum speed of the robot (needed, as the outdoor buggys can't handle velocities below about 0.3).
        float min_velocity_;
        //! Maximum velocity (to prevent the high level control from running amok).
        float max_velocity_;

        //! Maximum distance the robot is allowed to depart from the path. If this threshold is exceeded,
        //! the path follower will abort.
        double max_distance_to_path_;

        float velocity_; //FIXME: obsolete when RobotController is working

        double steer_slow_threshold_;

        //! Width of the collisin box for obstacle avoidance.
        float collision_box_width_;
        //! Minimum length of the collision box for obstacle avoidance (grows with increasing velocity).
        float collision_box_min_length_;
        float collision_box_crit_length_;
        //! Maximum length of the collision box for obstacle avoidance.
        float collision_box_max_length_;
        //! This factor determines, how much the length of the box is increased, depending on the velocity.
        float collision_box_velocity_factor_;
        //! The velocity for which the maximum length should be used
        float collision_box_velocity_saturation_;

        int path_idx;
        int wp_idx;
    };

    struct NullBehaviour {
    };

    class Behaviour
    {
    protected:
        Behaviour(BehaviouralPathDriver& parent)
            : parent_(parent), controller_(parent.getController())
        {}
        Path& getSubPath(unsigned index);
        int getSubPathCount() const;
        PathFollower& getNode();

        Command& getCommand();
        VectorFieldHistogram& getVFH();
        Options& getOptions();
        double distanceTo(const Waypoint& wp);

    public:
        virtual ~Behaviour() {}

        virtual void execute(int* status) = 0;

    protected:
        BehaviouralPathDriver& parent_;
        RobotController* controller_;
    };


public:
    BehaviouralPathDriver(ros::Publisher& cmd_pub, PathFollower *node);

    virtual void start();
    virtual void stop();
    virtual int getType();

    /**
     * @brief execute
     * @param feedback Feedback of the running path execution. Only meaningful, if return value is 1.
     * @param result Result of the finished path execution. Only meaningful, if return value is 0.
     * @return Returns 0 if the path execution is finished (no matter if successful or not) and 1 if it is still running.
     */
    virtual int execute(path_msgs::FollowPathFeedback& feedback, path_msgs::FollowPathResult& result);
    virtual void configure();
    virtual void setGoal(const path_msgs::FollowPathGoal& goal);

    PathFollower* getNode() const;

    void setPath(const nav_msgs::Path& path);
    bool checkCollision(double course);

    Vector3d getSlamPose() const
    {
        return slam_pose_;
    }

    const geometry_msgs::Pose &getSlamPoseMsg() const
    {
        return slam_pose_msg_;
    }

    Behaviour* getActiveBehaviour() const
    {
        return active_behaviour_;
    }

    const BehaviouralPathDriver::Options &getOptions() const
    {
        return options_;
    }

    RobotController* getController();

protected:
    void clearActive();
    void beep(const std::vector<int>& beeps);

private:
    PathFollower* node_;

    ros::NodeHandle private_nh_;

    ros::Publisher& cmd_pub_;
    ros::Publisher beeper_;

    Behaviour* active_behaviour_;

    Command current_command_;
    Options options_;

    Vector3d slam_pose_;
    geometry_msgs::Pose slam_pose_msg_;
    nav_msgs::Path path_;
    std::vector<Path> paths_;

    Visualizer* visualizer_;

    int pending_error_;
    ros::Time last_beep_;
    ros::Duration beep_pause_;
};

#endif // BEHAVIOURALPATHDRIVER_H
