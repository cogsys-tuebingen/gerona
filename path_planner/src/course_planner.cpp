/**
    Copyright (C) 2015 Karsten Bohlmann E&K AUTOMATION GmbH
    All rights reserved.

    @year   2015
    @author Karsten Bohlmann
    @email  bohlmann@gmail.com
    @file   course_planner.cpp


*/
#include <utils_path/geometry/circle.h>
#include <utils_path/geometry/line.h>
#include <utils_path/geometry/intersector.h>
#include <utils_path/geometry/tangentor.h>

#include "course_planner.h"

CoursePlanner::CoursePlanner()
{
    posearray_pub_ = nh.advertise<geometry_msgs::PoseArray>("static_poses",1000);

    avoidance_pub_ = nh.advertise<nav_msgs::Path>("avoidance_path",1000);

    start_pose_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 0, &CoursePlanner::startPoseCb, this);

    obstacle_pose_sub_ = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 0, &CoursePlanner::obstaclePoseCb, this);


    nh.param("resolution", resolution_, 0.1);
    nh.param("segments", segment_array_, segment_array_);
}


void CoursePlanner::addGeomPoses(const std::vector<path_geom::PathPose> &gposes, nav_msgs::Path &path)
{
    for (auto& gp : gposes) {
        path.poses.push_back(pathPose2Pose(gp));
    }
}


void CoursePlanner::obstaclePoseCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose)
{
    PathPose gp=pose2PathPose(pose->pose.pose);
    double obstacle_radius = 1.0;
    double avoidance_radius = 1.5;
    Circle obstacle(gp.pos_,obstacle_radius);
    vector<int> indices;
    findCircleOnCourse(obstacle,segments_,indices);

    if (indices.empty()) {
        // obstacle not relevant
        return;
    }
    // first and second segment might be the same
    auto& first_segment = segments_[indices.front()];
    const auto& first_line = std::dynamic_pointer_cast<Line>(first_segment);
    const auto& first_circle = std::dynamic_pointer_cast<Circle>(first_segment);
    std::vector<Circle> first_tangent_arcs;

    if (first_line) {
        Tangentor::tangentArc(*first_line, obstacle, avoidance_radius,first_tangent_arcs);
    } else if (first_circle) {
        Tangentor::tangentInnerArcs(*first_circle,obstacle,avoidance_radius,
                                    first_tangent_arcs);
    }
    if (first_tangent_arcs.size()>0) {
        std::cout <<" found "<< first_tangent_arcs.size() << " tangent arcs"<<std::endl;
    }

    auto& second_segment = segments_[indices.back()];
    const auto& second_line = std::dynamic_pointer_cast<Line>(second_segment);
    const auto& second_circle = std::dynamic_pointer_cast<Circle>(second_segment);
    std::vector<Circle> second_tangent_arcs;
    if (second_line) {
        Tangentor::tangentArc(obstacle,*second_line, avoidance_radius,second_tangent_arcs);
    } else if (first_circle) {
        Tangentor::tangentInnerArcs(*second_circle,obstacle,avoidance_radius,
                                    second_tangent_arcs);
    }
    if (second_tangent_arcs.size()>0) {
        std::cout <<" found "<< second_tangent_arcs.size() << " tangent arcs"<<std::endl;
    }







    // calculate the tangent arc from first segment to obstacle circle


    // tangent arc from obstacle circle to second segment

    // create the avoidance course as copy of original course with avoidance arcs


    // publish path


}


void CoursePlanner::createCourse(XmlRpc::XmlRpcValue &segment_array, const geometry_msgs::Pose& start_pose, nav_msgs::Path &path)
{
    if(segment_array.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR_STREAM("segments type is not array: " << segment_array.toXml());
        return;
    }
    path_geom::PathPose gp=pose2PathPose(start_pose);
    for(int i =0; i < segment_array.size(); i++) {
        std::vector<path_geom::PathPose> gposes;

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
            segments_.push_back(std::make_shared<path_geom::Line>(line));
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
                segments_.push_back(std::make_shared<path_geom::Circle>(arc));
                ROS_INFO("ARC FORWARD circle radius %f angle %f number of points %ld",radius, arc_angle,gposes.size());
                ROS_INFO("circle center is %f %f",arc.center().x(),arc.center().y());
            } else {
                path_geom::Circle arc= path_geom::Circle::createArcTo(gp,radius,arc_angle,arc_direction);
                arc.toPoses(resolution_,gposes,move_direction);
                segments_.push_back(std::make_shared<path_geom::Circle>(arc));
                ROS_INFO("circle radius %f angle %f number of points %ld",radius, arc_angle,gposes.size());
                ROS_INFO("circle center is %f %f",arc.center().x(),arc.center().y());
            }
            addGeomPoses(gposes,path);
            gp = gposes.back();
        }
    }

}



void CoursePlanner::startPoseCb(const geometry_msgs::PoseStampedConstPtr &pose_stamped)
{

    nav_msgs::Path path_raw = nav_msgs::Path();


    path_raw.header.frame_id = "map";
    path_raw.header.stamp = ros::Time::now();
    geometry_msgs::PoseArray poses;
    geometry_msgs::Pose pose;
    pose=pose_stamped->pose;
    createCourse(segment_array_,pose, path_raw);

    path_ = postprocess(path_raw);
    publish(path_, path_raw);
    for (vector<geometry_msgs::PoseStamped>::iterator pose_it=path_.poses.begin();pose_it!=path_.poses.end();++pose_it) {
        geometry_msgs::Pose pose;
        pose.position = pose_it->pose.position;
        pose.orientation = pose_it->pose.orientation;

        poses.poses.push_back(pose);
    }
    poses.header.frame_id="/map";
    posearray_pub_.publish(poses);


}




void CoursePlanner::execute(const path_msgs::PlanPathGoalConstPtr &goal)
{
    nav_msgs::Path path_raw = nav_msgs::Path();


    //        pose = tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(0,0,0));

    path_raw.header.frame_id = "map";
    path_raw.header.stamp = ros::Time::now();
    geometry_msgs::PoseArray poses;
    createCourse(segment_array_,goal->goal.pose, path_raw);



    path_ = postprocess(path_raw);
    publish(path_, path_raw);
    for (vector<geometry_msgs::PoseStamped>::iterator pose_it=path_.poses.begin();pose_it!=path_.poses.end();++pose_it) {
        geometry_msgs::Pose pose;
        pose.position = pose_it->pose.position;
        pose.orientation = pose_it->pose.orientation;

        poses.poses.push_back(pose);
    }
    poses.header.frame_id="/map";
    posearray_pub_.publish(poses);
    feedback(path_msgs::PlanPathFeedback::STATUS_DONE);

    path_msgs::PlanPathResult success;
    success.path = path_;
    server_.setSucceeded(success);
}


void CoursePlanner::findCircleOnCourse(const Circle &obstacle, const vector<shared_ptr<Shape> > &course,
                                       vector<int> &indices)
{
    indices.clear();

    unsigned ipoint_count = 0;
    // go over all segments in course and check for intersection with obstacle circle
    for (size_t i=0; i <course.size();++i) {
        auto& segment = course[i];

        vector<Vector2d> ipoints;

        auto line = dynamic_pointer_cast<Line>(segment);
        auto circle = dynamic_pointer_cast<Circle>(segment);
        if (line) {
            Intersector::intersect(*line, obstacle,ipoints);
        } else if (circle) {
            Intersector::intersect(*circle, obstacle,ipoints);
        } // no final else ..all other cases produce no interscetion points

        if (ipoints.size()>0) {
            indices.push_back((int)i);
            ipoint_count+=ipoints.size();
        }
        if (ipoint_count>=2) {
            break;
        }
    }
}


