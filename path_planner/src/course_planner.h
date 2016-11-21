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
#include "course/course_map.h"
#include "course/search.h"

#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>

#include <cslibs_path_planning/geometry/shape.h>
#include <cslibs_path_planning/geometry/circle.h>

class CoursePlanner : public Planner
{
public:
    CoursePlanner();

    virtual bool supportsGoalType(int type) const override;

    void execute(const path_msgs::PlanPathGoalConstPtr &goal);

    virtual path_msgs::PathSequence plan (const path_msgs::PlanPathGoal &goal,
                                          const lib_path::Pose2d& from_world, const lib_path::Pose2d& to_world,
                                          const lib_path::Pose2d& from_map, const lib_path::Pose2d& to_map) {

        throw std::logic_error("should not be called");
    }

    void tick();

    void addCurve(double angle, double radius);

    void startPoseCb(const geometry_msgs::PoseStampedConstPtr &pose);

    void obstaclePoseCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose);



    void planAvoidanceCb (const path_msgs::PlanAvoidanceGoalConstPtr &goal);

    void findCircleOnCourse(const path_geom::Circle& obstacle, const std::vector<std::shared_ptr<path_geom::Shape>>& course, std::vector<int>& indices );

    void findPosOnCourse(const path_geom::PathPose& gp, const std::vector<std::shared_ptr<path_geom::Shape>>& course,
                        int& nearest_idx,Eigen::Vector2d& nearest);
private:
    double resolution_ = 0.1;
    double avoidance_radius_ = 1.5;
    double obstacle_radius_ = 1.5;

    XmlRpc::XmlRpcValue segment_array_;
    XmlRpc::XmlRpcValue map_segment_array_;

    CourseMap course_;
    Search course_search_;

    std::vector<std::shared_ptr<path_geom::Shape>> course_segments_;
    std::vector<std::shared_ptr<path_geom::Shape>> active_segments_;
    path_msgs::PathSequence avoidance_path_;

    path_msgs::PathSequence path_;
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
                              path_msgs::PathSequence& path);



    /**
     * @brief creates a course/static path from geometry input
     * @param segment_array
     * @param pose
     * @param path
     */
    void createCourse(XmlRpc::XmlRpcValue& segment_array, const geometry_msgs::Pose& pose,
                      std::vector<std::shared_ptr<path_geom::Shape>>& segments,path_msgs::PathSequence &path);

    void segments2Path(const std::vector<std::shared_ptr<path_geom::Shape>>& segments,double angle_offset, int direction ,
                       path_msgs::PathSequence& path );

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
        pose.header.frame_id="map";
        return pose;
    }
    void addGeomPoses(const path_geom::PathPoseVec& gposes, path_msgs::PathSequence& path);


    /**
      evil hack should be provided by feedback of follower
      */
    bool getWorldPose(const std::string& world_frame, const std::string& robot_frame,geometry_msgs::PoseStamped& result) const;
    tf::TransformListener pose_listener_;


    path_msgs::PathSequence planWithStaticPath(const path_msgs::PlanPathGoalConstPtr &goal);
    path_msgs::PathSequence planWithMap(const path_msgs::PlanPathGoalConstPtr &goal);
};

#endif // COURSE_PLANNER_H
