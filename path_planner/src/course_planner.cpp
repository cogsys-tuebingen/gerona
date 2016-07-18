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

    : course_(nh),
      plan_avoidance_server_(nh, "/plan_avoidance", boost::bind(&CoursePlanner::planAvoidanceCb, this, _1), false)
{
    posearray_pub_ = nh.advertise<geometry_msgs::PoseArray>("static_poses",1000);

    avoidance_pub_ = nh.advertise<nav_msgs::Path>("avoidance_path",1000);

    start_pose_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal2", 0, &CoursePlanner::startPoseCb, this);

    obstacle_pose_sub_ = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/obst_pose", 0, &CoursePlanner::obstaclePoseCb, this);

    ros::NodeHandle pnh("~");

    pnh.param("avoidance_radius",avoidance_radius_,1.5);
    pnh.param("obstacle_radius",obstacle_radius_,1.0);

    pnh.param("resolution", resolution_, 0.1);
    pnh.param("segments", segment_array_, segment_array_);
    pnh.param("map_segments", map_segment_array_, map_segment_array_);
    
    plan_avoidance_server_.start();

    course_.createMap(map_segment_array_);
}

void CoursePlanner::tick()
{
    course_.publishMarkers();
}

bool CoursePlanner::supportsGoalType(int type) const
{
    return type == path_msgs::Goal::GOAL_TYPE_POSE;
}


void CoursePlanner::addGeomPoses(const PathPoseVec &gposes, nav_msgs::Path &path)
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
    path_geom::PathPose obstacle_gp;
    obstacle_gp.pos_.x()=obstacle.position.x;
    obstacle_gp.pos_.y()=obstacle.position.y;
    //path_geom::PathPose robot_gp=pose2PathPose(current.pose);
    geometry_msgs::PoseStamped robot_pose;
    bool has_pose = getWorldPose("/map","/base_link",robot_pose);
    if (!has_pose) {
        ROS_ERROR("cannot find robot pose with TF");
        return;
    }

    PathPose robot_gp=pose2PathPose(robot_pose.pose);



    nav_msgs::Path path;
    processPlanAvoidance(obstacle_gp, robot_gp, path);
    path_msgs::PlanAvoidanceResult success;
    success.path = path;
    plan_avoidance_server_.setSucceeded(success);
    //path_publisher_.publish(path);
    avoidance_pub_.publish(path);

}


void CoursePlanner::processPlanAvoidance(const PathPose &obstacle_gp, const PathPose &robot_gp,
                                         nav_msgs::Path& path)
{

    Circle obstacle(obstacle_gp.pos_,obstacle_radius_);
    Circle ext_obstacle(obstacle_gp.pos_,obstacle_radius_+avoidance_radius_);
    vector<int> indices;
    findCircleOnCourse(ext_obstacle,active_segments_,indices);
    int robot_idx;
    Eigen::Vector2d nearest;
    findPosOnCourse(robot_gp,active_segments_, robot_idx, nearest);
    if (indices.empty()) {
        // obstacle not relevant
        return;
    }

    std::vector<std::shared_ptr<path_geom::Shape>> avoidance_path1, avoidance_path2, avoidance_path;

    ROS_INFO("NEW Obstacle center %f %f",obstacle_gp.pos_.x(),obstacle_gp.pos_.y());
    // first and second segment might be the same
    int first_idx = indices.front();
    for (int i=0;i<2;++i) {
        if (first_idx<0) {
            first_idx = active_segments_.size()-1;
        }
        auto& first_segment = active_segments_[first_idx];
        Tangentor::tangentPath(first_segment,obstacle,avoidance_radius_,true,avoidance_path1);
        if (avoidance_path1.size()==3) {
            break;
        }
        --first_idx;
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
    int second_idx = indices.back();
    for (int i=0;i<3;++i) {

        if (second_idx>=(int)active_segments_.size()) {
            second_idx = 0;
        }
        auto& second_segment = active_segments_[second_idx];
        Tangentor::tangentPath(second_segment,*obstacle_circle,avoidance_radius_,false,avoidance_path2);
        if (avoidance_path2.size()==3) {
            break;
        }
        ++second_idx;
    }
    if (avoidance_path2.size()!=3) {
        ROS_INFO("failed to find adequate avoidance course from obstacle back to course");
        return;
    }
    ROS_INFO(" path size found %ld ",avoidance_path2.size());
    nav_msgs::Path path_raw;
    // remove last elememnt from first path part as it is contained in second path
    avoidance_path1.pop_back();

    // cerate the complete path from robot to finish


    int idx = robot_idx;
    int cnt = active_segments_.size();
    while (idx!=first_idx && idx<(int)active_segments_.size()) {
        avoidance_path.push_back(active_segments_[idx]);
        ++idx;
        --cnt;
    }

    avoidance_path.insert(avoidance_path.end(),avoidance_path1.begin(), avoidance_path1.end());
    avoidance_path.insert(avoidance_path.end(),avoidance_path2.begin(), avoidance_path2.end());

    bool success =avoidance_path.front()->selectStartPoint(nearest);
    ROS_INFO_STREAM("ronot pos is "<<robot_gp.pos_);

    ROS_INFO_STREAM("original line is "<<avoidance_path.front()->startPoint().x()<< " "
                    <<avoidance_path.front()->startPoint().y());
    if (!success) {
        ROS_INFO_STREAM("failed tos et start point "<<nearest.x()<< " "<< nearest.y());

    } else {
        ROS_INFO_STREAM("set start point "<<nearest.x()<< " "<< nearest.y());
    }
    // add remaining parts of original course

    for (int idx=second_idx+1;idx<(int)active_segments_.size();++idx) {
        ROS_INFO("adding segment %u",idx);
        avoidance_path.push_back(active_segments_[idx]);
    }

    active_segments_ = avoidance_path;

    for (auto &s: avoidance_path) {
        PathPoseVec gposes;
        s->toPoses(resolution_,gposes,path_geom::FORWARD,false);
        addGeomPoses(gposes, path_raw);
    }
    ROS_INFO("generated path has %lu poses",path_raw.poses.size());
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
                                  double angle_offset, int direction,nav_msgs::Path &path)
{
   PathPoseVec gposes;
   for (auto& segment : segments) {
       segment->toPoses(resolution_,gposes, direction);
       addGeomPoses(gposes, path);
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
    createCourse(segment_array_,pose,course_segments_, path_raw);
    active_segments_ = course_segments_;
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

    if(course_.hasSegments()) {
        path_raw = planWithMap(goal);
    } else {
        path_raw = planWithStaticPath(goal);
    }

    path_raw.header.frame_id = "map";
    path_raw.header.stamp = ros::Time::now();

    path_ = postprocess(path_raw);

    publish(path_, path_raw);
    geometry_msgs::PoseArray poses;
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




nav_msgs::Path CoursePlanner::planWithMap(const path_msgs::PlanPathGoalConstPtr &goal)
{
    nav_msgs::Path path_raw = nav_msgs::Path();

    geometry_msgs::PoseStamped robot_pose;
    bool has_pose = getWorldPose("/map","/base_link",robot_pose);
    if (!has_pose) {
        ROS_ERROR("cannot find robot pose with TF");
        return path_raw;
    }

    std::vector<path_geom::PathPose> pts = course_.findPath(pose2PathPose(robot_pose.pose), pose2PathPose(goal->goal.pose.pose));
    for(const path_geom::PathPose& pose : pts) {
        path_raw.poses.push_back(pathPose2Pose(pose));
    }

    return path_raw;
}



nav_msgs::Path CoursePlanner::planWithStaticPath(const path_msgs::PlanPathGoalConstPtr &goal)
{
    nav_msgs::Path path_raw = nav_msgs::Path();

    path_geom::PathPose goal_gp = pose2PathPose(goal->goal.pose.pose);
    int nearest_idx;
    Eigen::Vector2d nearest;
    findPosOnCourse(goal_gp,course_segments_,nearest_idx,nearest);
    if ((nearest_idx>=0) && (goal_gp.pos_-nearest).norm()<0.5) {
        active_segments_.clear();
        shared_ptr<Shape> nearest_cpy=Shape::deepCopy(course_segments_[nearest_idx]);
        nearest_cpy->selectStartPoint(nearest);

        active_segments_.push_back(nearest_cpy);
        int idx=nearest_idx+1;

        int cnt = (int)course_segments_.size();
        while (idx!=nearest_idx && cnt) {
            if (idx>=(int)course_segments_.size()) {
                idx=0;
            }
            active_segments_.push_back(course_segments_[idx]);
            idx+=1;
            --cnt;
        }
        /*
        active_segments_.push_back(course_segments_[nearest_idx]);

        active_segments_.back()->selectEndPoint(nearest);*/
        segments2Path(active_segments_,0.0,path_geom::FORWARD,path_raw);
        ROS_INFO_STREAM("resuning course with "<<active_segments_.size()<< " segments");
    } else {

        ROS_INFO("creating course");
        createCourse(segment_array_,goal->goal.pose.pose, course_segments_,path_raw);
        active_segments_ = course_segments_;
    }

    return path_raw;
}


void CoursePlanner::findPosOnCourse(const PathPose &gp, const vector<shared_ptr<Shape> > &course, int &nearest_idx,
                                    Eigen::Vector2d &nearest)
{
    if (course.empty()) {
        nearest_idx = -1;
        return;
    }
    std::vector<double> distances;
    distances.reserve(course.size());
    for (auto& segment : course) {
        distances.push_back(segment->distanceTo(gp.pos_));
    }
    auto dist_it = min_element(begin(distances),end(distances));

    nearest_idx = dist_it-distances.begin();
    nearest = course[nearest_idx]->nearestPointTo(gp.pos_);

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

