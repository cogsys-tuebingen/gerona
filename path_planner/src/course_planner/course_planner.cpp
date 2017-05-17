/**
    Copyright (C) 2015 Karsten Bohlmann E&K AUTOMATION GmbH
    All rights reserved.

    @year   2015
    @author Karsten Bohlmann
    @email  bohlmann@gmail.com
    @file   course_planner.cpp


*/

#include <cslibs_path_planning/geometry/circle.h>

#include <cslibs_path_planning/geometry/line.h>
#include <cslibs_path_planning/geometry/intersector.h>
#include <cslibs_path_planning/geometry/tangentor.h>

#include "course_planner.h"

using namespace std;
using namespace Eigen;
using namespace path_geom;

CoursePlanner::CoursePlanner()

    : course_(nh),
      course_search_(course_)
{
    posearray_pub_ = nh.advertise<geometry_msgs::PoseArray>("static_poses",1000);

    ros::NodeHandle pnh("~");

    pnh.param("course/resolution", resolution_, 0.1);
    pnh.param("course/map_segments", map_segment_array_, map_segment_array_);

    course_.load(map_segment_array_);
}

void CoursePlanner::tick()
{
    course_.publishMarkers();
}

bool CoursePlanner::supportsGoalType(int type) const
{
    return type == path_msgs::Goal::GOAL_TYPE_POSE;
}


void CoursePlanner::addGeomPoses(const PathPoseVec &gposes, path_msgs::PathSequence &path_sequence)
{
    if(path_sequence.paths.empty()) {
        path_sequence.paths.emplace_back();
    }

    auto& path = path_sequence.paths.back();
    for (auto& gp : gposes) {
        path.poses.push_back(pathPose2Pose(gp));
    }
}


void CoursePlanner::createCourse(XmlRpc::XmlRpcValue &segment_array, const geometry_msgs::Pose& start_pose,
                                 vector<shared_ptr<Shape>>& segments,
                                 path_msgs::PathSequence &path)
{
    if(segment_array.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR_STREAM("segments type is not array: " << segment_array.toXml());
        return;
    }
    // clean up
    segments.clear();
    path.paths.clear();

    path_geom::PathPose gp=pose2PathPose(start_pose);
    for(int i =0; i < segment_array.size(); i++) {
        PathPoseVec gposes;

        XmlRpc::XmlRpcValue& segment = segment_array[i];
        if(segment.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_ERROR_STREAM("segment type is not array: " << segment.toXml());
            return;
        }


        if(segment.size() == 2) {
            double speed =  segment[0];
            int direction = path_geom::FORWARD;
            if (speed<0) {
                direction = path_geom::BACKWARD;
            }
            path_geom::Line line(gp,segment[1],direction);
            //segments.push_back(std::make_shared<path_geom::Line>(line));
            segments.push_back(std::allocate_shared<path_geom::Line>(Eigen::aligned_allocator<Line>(),line));
            line.toPoses(resolution_,gposes,direction,false);
            addGeomPoses(gposes,path);
            gp = gposes.back();

        } else if (segment.size() == 3) {
            double speed =  segment[0];
            int move_direction = path_geom::FORWARD;
            if (speed<0) {
                move_direction = path_geom::BACKWARD;
            }
            double radius = fabs((double)segment[2]);
            double arc_angle = segment[1];
            int arc_direction;
            if (((double)segment[2])<0) {
                arc_direction = path_geom::ARC_RIGHT;
            } else {
                arc_direction = path_geom::ARC_LEFT;
            }
            if (move_direction==path_geom::FORWARD) {
                path_geom::Circle arc= path_geom::Circle::createArcFrom(gp,radius,arc_angle,arc_direction);

                arc.toPoses(resolution_,gposes,move_direction);
                //segments.push_back(std::make_shared<path_geom::Circle>(arc));
                segments.push_back(std::allocate_shared<path_geom::Circle>(Eigen::aligned_allocator<Circle>(),arc));

                ROS_INFO("ARC FORWARD circle radius %f angle %f number of points %lu",radius, arc_angle,gposes.size());
                ROS_INFO("circle center is %f %f",arc.center().x(),arc.center().y());
            } else {
                path_geom::Circle arc= path_geom::Circle::createArcTo(gp,radius,arc_angle,arc_direction);
                arc.toPoses(resolution_,gposes,move_direction);
                //segments.push_back(std::make_shared<path_geom::Circle>(arc));
                segments.push_back(std::allocate_shared<path_geom::Circle>(Eigen::aligned_allocator<Circle>(),arc));

                ROS_INFO("circle radius %f angle %f number of points %lu",radius, arc_angle,gposes.size());
                ROS_INFO("circle center is %f %f",arc.center().x(),arc.center().y());
            }
            addGeomPoses(gposes,path);
            gp = gposes.back();
        }
    }

}


void CoursePlanner::segments2Path(const vector<shared_ptr<Shape> > &segments,
                                  double angle_offset, int direction,path_msgs::PathSequence &path)
{
   PathPoseVec gposes;
   for (auto& segment : segments) {
       segment->toPoses(resolution_,gposes, direction);
       addGeomPoses(gposes, path);
   }
}



path_msgs::PathSequence CoursePlanner::plan (const path_msgs::PlanPathGoal &goal,
                   const lib_path::Pose2d& from_world, const lib_path::Pose2d& to_world,
                   const lib_path::Pose2d& from_map, const lib_path::Pose2d& to_map)
{

    if(!course_.hasSegments()) {
        return {};
    }

    path_geom::PathPose from_world_p(from_world.x, from_world.y, from_world.theta);
    path_geom::PathPose to_world_p(to_world.x, to_world.y, to_world.theta);

    auto res = course_search_.findPath(map_info, goal, from_world_p, to_world_p);
    res.header = goal.goal.pose.header;

    return res;
}
