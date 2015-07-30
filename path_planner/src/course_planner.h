/**
    Copyright (C) 2015 Karsten Bohlmann E&K AUTOMATION GmbH
    All rights reserved.

    @year   2015
    @author Karsten Bohlmann
    @email  bohlmann@gmail.com
    @file   course_planner.h


*/

#ifndef COURSE_PLANNER_H
#define COURSE_PLANNER_H

#include "planner_node.h"
#include <memory>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <path_msgs/PlanAvoidanceAction.h>

#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>

#include <utils_path/geometry/shape.h>
#include <utils_path/geometry/circle.h>
using namespace std;
using namespace path_geom;
class CoursePlanner : public Planner
{
public:
    CoursePlanner();

    void execute(const path_msgs::PlanPathGoalConstPtr &goal);

    nav_msgs::Path plan (const geometry_msgs::PoseStamped &goal,
                         const lib_path::Pose2d& from_world, const lib_path::Pose2d& to_world,
                         const lib_path::Pose2d& from_map, const lib_path::Pose2d& to_map) {

        throw std::logic_error("should not be called");
    }

    void addCurve(double angle, double radius);

    void startPoseCb(const geometry_msgs::PoseStampedConstPtr &pose);

    void obstaclePoseCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose);



    void planAvoidanceCb (const path_msgs::PlanAvoidanceGoalConstPtr &goal);

    void findCircleOnCourse(const Circle& obstacle, const vector<shared_ptr<Shape>>& course, vector<int>& indices );

    void findPosOnCourse(const path_geom::PathPose& gp, const vector<shared_ptr<Shape>>& course,
                        int& nearest_idx,Eigen::Vector2d& nearest);
private:
    double resolution_ = 0.1;
    double avoidance_radius_ = 1.5;
    double obstacle_radius_ = 1.5;

    XmlRpc::XmlRpcValue segment_array_;

    vector<shared_ptr<Shape>> course_segments_;
    vector<shared_ptr<Shape>> active_segments_;
    nav_msgs::Path avoidance_path_;

    nav_msgs::Path path_;
    tf::Transform pose_;
    ros::Publisher posearray_pub_;
    ros::Publisher avoidance_pub_;
    ros::Subscriber start_pose_sub_;
    ros::Subscriber obstacle_pose_sub_;

    typedef actionlib::SimpleActionServer<path_msgs::PlanAvoidanceAction> PlanAvoidanceServer;

    //! Action server that communicates with the guidance control
    PlanAvoidanceServer plan_avoidance_server_;

    void processPlanAvoidance(const path_geom::PathPose& obstacle,
                              const path_geom::PathPose& robot,
                              nav_msgs::Path& path);


    /**
     * @brief creates a course/static path from geometry input
     * @param segment_array
     * @param pose
     * @param path
     */
    void createCourse(XmlRpc::XmlRpcValue& segment_array, const geometry_msgs::Pose& pose,
                      vector<shared_ptr<Shape>>& segments,nav_msgs::Path &path);

    void segments2Path(const vector<shared_ptr<Shape>>& segments,double angle_offset, int direction ,
                       nav_msgs::Path& path );

    path_geom::PathPose pose2PathPose(const geometry_msgs::Pose& pose) {
        return path_geom::PathPose(pose.position.x,pose.position.y,
                                   tf::getYaw(pose.orientation));
    }
    geometry_msgs::PoseStamped pathPose2Pose(const path_geom::PathPose& gp) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x=gp.pos_.x();
        pose.pose.position.y=gp.pos_.y();
        pose.pose.position.z=0.0;
        pose.pose.orientation=tf::createQuaternionMsgFromYaw(gp.theta_);
        // ***todo hack
        pose.header.frame_id="/map";
        return pose;
    }
    void addGeomPoses(const PathPoseVec& gposes, nav_msgs::Path& path);


    /**
      evil hack should be provided by feedback of follower
      */
    bool getWorldPose(const std::string& world_frame, const std::string& robot_frame,geometry_msgs::PoseStamped& result) const;
    tf::TransformListener pose_listener_;


};

#endif // COURSE_PLANNER_H
