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

    pnh.param("resolution", resolution_, 0.1);
    pnh.param("segments", segment_array_, segment_array_);
    pnh.param("map_segments", map_segment_array_, map_segment_array_);

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




void CoursePlanner::execute(const path_msgs::PlanPathGoalConstPtr &goal)
{
    path_msgs::PathSequence path_raw = path_msgs::PathSequence();

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
    for(const path_msgs::DirectionalPath& path : path_raw.paths) {
        for (const geometry_msgs::PoseStamped& spose : path.poses) {
            poses.poses.push_back(spose.pose);
        }
    }
    poses.header.frame_id="map";
    posearray_pub_.publish(poses);

    feedback(path_msgs::PlanPathFeedback::STATUS_DONE);

    path_msgs::PlanPathResult success;
    success.path = path_;
    server_.setSucceeded(success);
}




path_msgs::PathSequence CoursePlanner::planWithMap(const path_msgs::PlanPathGoalConstPtr &goal)
{
    path_msgs::PathSequence path_raw = path_msgs::PathSequence();

    geometry_msgs::PoseStamped robot_pose;
    bool has_pose = getWorldPose("map","base_link",robot_pose);
    if (!has_pose) {
        ROS_ERROR("cannot find robot pose with TF");
        return path_raw;
    }

    std::vector<path_geom::PathPose> pts = course_search_.findPath(pose2PathPose(robot_pose.pose), pose2PathPose(goal->goal.pose.pose));

    path_raw.paths.emplace_back();
    auto& path = path_raw.paths.back();
    for(const path_geom::PathPose& pose : pts) {
        path.poses.push_back(pathPose2Pose(pose));
    }

    return path_raw;
}



path_msgs::PathSequence CoursePlanner::planWithStaticPath(const path_msgs::PlanPathGoalConstPtr &goal)
{
    path_msgs::PathSequence path_raw = path_msgs::PathSequence();

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
    bool has_pose = getWorldPose("map","base_link",here);
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

