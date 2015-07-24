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
    :    plan_avoidance_server_(nh, "/plan_avoidance", boost::bind(&CoursePlanner::planAvoidanceCb, this, _1), false)

{
    posearray_pub_ = nh.advertise<geometry_msgs::PoseArray>("static_poses",1000);

    avoidance_pub_ = nh.advertise<nav_msgs::Path>("avoidance_path",1000);

    start_pose_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 0, &CoursePlanner::startPoseCb, this);

    obstacle_pose_sub_ = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/obst_pose", 0, &CoursePlanner::obstaclePoseCb, this);


    nh.param("avoidance_radius",avoidance_radius_,1.5);
    nh.param("obstacle_radius",obstacle_radius_,1.0);

    nh.param("resolution", resolution_, 0.1);
    nh.param("segments", segment_array_, segment_array_);
    plan_avoidance_server_.start();
}


void CoursePlanner::addGeomPoses(const std::vector<path_geom::PathPose> &gposes, nav_msgs::Path &path)
{
    for (auto& gp : gposes) {
        path.poses.push_back(pathPose2Pose(gp));
    }
}


void CoursePlanner::obstaclePoseCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose)
{
    PathPose obstacle_gp=pose2PathPose(pose->pose.pose);
    geometry_msgs::PoseStamped robot_pose;
    bool has_pose = getWorldPose("/map","/base_link",robot_pose);
    if (!has_pose) {
        ROS_ERROR("cannot find robot pose with TF");
        return;
    }

    PathPose robot_gp=pose2PathPose(robot_pose.pose);
    nav_msgs::Path smoothed_path;
    processPlanAvoidance(obstacle_gp, robot_gp, smoothed_path);


    avoidance_pub_.publish(smoothed_path);


}


void CoursePlanner::planAvoidanceCb(const path_msgs::PlanAvoidanceGoalConstPtr &goal_msg)
{
    ROS_INFO("Avoidance planner received request for avoidance plan");
    const path_msgs::Obstacle& obstacle = goal_msg->obstacle;
    //const geometry_msgs::PoseStamped& goal = goal_msg->goal;
    const geometry_msgs::PoseStamped& current = goal_msg->current;
    path_geom::PathPose obstacle_gp;
    obstacle_gp.pos_.x()=obstacle.position.x;
    obstacle_gp.pos_.y()=obstacle.position.y;
    path_geom::PathPose robot_gp=pose2PathPose(current.pose);
    nav_msgs::Path path;
    processPlanAvoidance(obstacle_gp, robot_gp, path);
    path_msgs::PlanAvoidanceResult success;
    success.path = path;
    plan_avoidance_server_.setSucceeded(success);
    path_publisher_.publish(path);
}


void CoursePlanner::processPlanAvoidance(const PathPose &obstacle_gp, const PathPose &robot_gp,
                                         nav_msgs::Path& path)
{

    Circle obstacle(obstacle_gp.pos_,obstacle_radius_);
    Circle ext_obstacle(obstacle_gp.pos_,obstacle_radius_+avoidance_radius_);
    vector<int> indices;
    findCircleOnCourse(ext_obstacle,segments_,indices);

    if (indices.empty()) {
        // obstacle not relevant
        return;
    }

    std::vector<std::shared_ptr<path_geom::Shape>> avoidance_path1, avoidance_path2;

    ROS_INFO("NEW Obstacle center %f %f",obstacle_gp.pos_.x(),obstacle_gp.pos_.y());
    // first and second segment might be the same
    auto& first_segment = segments_[indices.front()];
    auto first_line = std::dynamic_pointer_cast<Line>(first_segment);
    auto first_circle = std::dynamic_pointer_cast<Circle>(first_segment);
    std::vector<Circle> first_tangent_arcs;

    if (first_line) {
        ROS_INFO_STREAM("first segment "<<*first_line);
        Tangentor::tangentPath(*first_line,obstacle,avoidance_radius_, avoidance_path1);

    } else if (first_circle) {
        ROS_INFO("first segment circle center %f %f", first_circle->center().x(),first_circle->center().y());
        Tangentor::tangentPath(obstacle,*first_circle,avoidance_radius_,false,
                                    avoidance_path1);
    }

    std::cout <<" found "<< avoidance_path1.size() << " tangent arcs"<<std::endl;
    if (avoidance_path1.size()!=3) {
        ROS_INFO("failed to find adequate avoidance course");
        return;
    }
    auto obstacle_circle = std::dynamic_pointer_cast<Circle>(avoidance_path1[2]);
    if (!obstacle_circle) {
        ROS_INFO("fail in obstacle avoidance");
        return;
    }
    obstacle_circle->setArcAngle(2*M_PI);

    auto& second_segment = segments_[indices.back()];
    auto second_line = std::dynamic_pointer_cast<Line>(second_segment);
    auto second_circle = std::dynamic_pointer_cast<Circle>(second_segment);
    std::vector<Circle> second_tangent_arcs;
    if (second_line) {

        Tangentor::tangentPath(*obstacle_circle,*second_line,avoidance_radius_, avoidance_path2);
    } else if (second_circle) {
        ROS_INFO("second segment circle center %f %f", second_circle->center().x(),second_circle->center().y());
        Tangentor::tangentPath(*obstacle_circle,*second_circle,avoidance_radius_,true,
                                    avoidance_path2);
    }
    ROS_INFO(" path size found %ld ",avoidance_path2.size());
    nav_msgs::Path path_raw;
    // remove last elememnt from first path part as it is contained in second path
    avoidance_path1.pop_back();
    avoidance_path1.insert(avoidance_path1.end(),avoidance_path2.begin(), avoidance_path2.end());

    // add remaining parts of original course
    for (unsigned idx=indices.back()+1;idx<segments_.size();++idx) {
        avoidance_path1.push_back(segments_[idx]);
    }

    for (auto &s: avoidance_path1) {
        std::vector<path_geom::PathPose> gposes;
        s->toPoses(resolution_,gposes,path_geom::FORWARD,false);
        addGeomPoses(gposes, path_raw);
    }
    ROS_INFO("generated path has %ld poses",path_raw.poses.size());
    path_raw.header.frame_id = "map";
    path_raw.header.stamp = ros::Time::now();
    path = postprocess(path_raw);



}


void CoursePlanner::createCourse(XmlRpc::XmlRpcValue &segment_array, const geometry_msgs::Pose& start_pose,
                                 vector<shared_ptr<Shape>>& segments,
                                 nav_msgs::Path &path)
{
    if(segment_array.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR_STREAM("segments type is not array: " << segment_array.toXml());
        return;
    }
    // clean up
    segments.clear();
    path.poses.clear();

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
            segments.push_back(std::make_shared<path_geom::Line>(line));
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
                segments.push_back(std::make_shared<path_geom::Circle>(arc));
                ROS_INFO("ARC FORWARD circle radius %f angle %f number of points %ld",radius, arc_angle,gposes.size());
                ROS_INFO("circle center is %f %f",arc.center().x(),arc.center().y());
            } else {
                path_geom::Circle arc= path_geom::Circle::createArcTo(gp,radius,arc_angle,arc_direction);
                arc.toPoses(resolution_,gposes,move_direction);
                segments.push_back(std::make_shared<path_geom::Circle>(arc));
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
    createCourse(segment_array_,pose,segments_, path_raw);

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
    createCourse(segment_array_,goal->goal.pose, segments_,path_raw);



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


    geometry_msgs::PoseStamped here;
    bool has_pose = getWorldPose("/map","/base_link",here);
    if (!has_pose) {
        ROS_ERROR("no pose for robot found");
        return;
    }
    double search_radius = 0.7;
    Eigen::Vector2d search_center(here.pose.position.x, here.pose.position.y);

    Circle agv_search(search_center,search_radius);

    size_t start_idx = 0;
    while (start_idx<course.size()) {
        auto& segment = course[start_idx];
        vector<Vector2d> ipoints;
        auto line = dynamic_pointer_cast<Line>(segment);
        auto circle = dynamic_pointer_cast<Circle>(segment);
        if (line) {
            Intersector::intersect(*line, agv_search,ipoints);
        } else if (circle) {
            Intersector::intersectArcs(*circle, agv_search,ipoints);
        } // no final else ..all other cases produce no interscetion points
        if (ipoints.size()>0) {
            break;
        }
        ++start_idx;
    }
    ROS_INFO_STREAM("agv pos "<<agv_search.center() << " on segment "<<start_idx);

    unsigned ipoint_count = 0;
    size_t idx = start_idx;
    // go over all segments in course and check for intersection with obstacle circle
    for (size_t i=0; i <course.size();++i) {
        if (idx>=course.size()) {
            idx = 0;
        }
        auto& segment = course[idx];

        vector<Vector2d> ipoints;
        ipoints.clear();
        auto line = dynamic_pointer_cast<Line>(segment);
        auto circle = dynamic_pointer_cast<Circle>(segment);
        if (line) {
            Intersector::intersect(*line, obstacle,ipoints);
        } else if (circle) {
            Intersector::intersectArcs(*circle, obstacle,ipoints);
        } // no final else ..all other cases produce no interscetion points

        if (ipoints.size()>0) {
            indices.push_back((int)idx);
            ipoint_count+=ipoints.size();
        }
        if (ipoint_count>=2) {
            break;
        }
        ++idx;
    }
}


bool CoursePlanner::getWorldPose(const std::string& world_frame, const std::string& robot_frame,geometry_msgs::PoseStamped& result) const
{
    tf::StampedTransform transform;
    geometry_msgs::TransformStamped msg;

    try {
        pose_listener_.lookupTransform(world_frame, robot_frame, ros::Time(0), transform);

    } catch (tf::TransformException& ex) {
        ROS_ERROR("error with transform robot pose: %s", ex.what());
        return false;
    }
    tf::transformStampedTFToMsg(transform, msg);

    result.pose.position.x = msg.transform.translation.x;
    result.pose.position.y = msg.transform.translation.y;
    result.pose.position.z = msg.transform.translation.z;
    result.pose.orientation = msg.transform.rotation;
    return true;
}

