/// COMPONENT
#include "planner_node.h"

/// SYSTEM
#include <math.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>

namespace {
geometry_msgs::PoseStamped tf2pose(const tf::Transform& pose)
{
    geometry_msgs::PoseStamped r;
    tf::poseTFToMsg(pose, r.pose);
    return r;
}
}

struct Segment
{
    double resolution;
    Segment(double resolution)
        : resolution(resolution)
    {}

    virtual ~Segment() {}
    virtual void add(tf::Transform& pose, nav_msgs::Path& path) = 0;
};

struct Curve : public Segment
{
    double angle;
    double radius;

    Curve(double angle, double radius, double resolution)
        : Segment(resolution), angle(angle), radius(radius)
    {}

    void add(tf::Transform& pose, nav_msgs::Path& path)
    {
        double span = 0;
        double sign = angle > 0 ? 1.0 : -1.0;
        double sign_delta_theta = radius>=0?1.0 : -1.0;

        do {
            double delta_theta = sign * std::min(std::abs(angle) - span, resolution);
            span += std::abs(delta_theta);

            double theta = tf::getYaw(pose.getRotation());
            tf::Vector3 delta_pos(radius * (-std::sin(theta) + std::sin(theta + delta_theta)),
                                  radius * (std::cos(theta) - std::cos(theta + delta_theta)),
                                  0);

            pose.setOrigin(pose.getOrigin() + sign*delta_pos);
            pose.setRotation(tf::createQuaternionFromYaw(theta + sign_delta_theta*delta_theta));

            path.poses.push_back(tf2pose(pose));

        } while(span < std::abs(angle));
    }
};

struct Straight : public Segment
{
    double length_;
    int dir_;
    Straight(double length, double resolution)
        : Segment(resolution), length_(fabs(length)),dir_(length>=0.0?1:-1)
    {

    }

    void add(tf::Transform& pose, nav_msgs::Path& path)
    {

        double distance = 0;
        do {
            double step = std::min(length_ - distance, resolution);
            tf::Transform delta(tf::createIdentityQuaternion(), tf::Vector3(dir_*step, 0, 0));
            distance += step;

            pose = pose * delta;

            path.poses.push_back(tf2pose(pose));

        } while(distance < length_);
    }
};

struct StaticPathPlanner : public Planner
{
    StaticPathPlanner()
    {
        posearray_pub_ = nh_priv.advertise<geometry_msgs::PoseArray>("static_poses",1000);
        nh_priv.param("resolution", resolution_, 0.1);
    }


    void execute(const path_msgs::PlanPathGoalConstPtr &goal)
    {
        nav_msgs::Path path_raw = nav_msgs::Path();

        tf::poseMsgToTF(goal->goal.pose, pose_);
//        pose = tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(0,0,0));

        path_raw.header.frame_id = "map";
        path_raw.header.stamp = ros::Time::now();
        geometry_msgs::PoseArray poses;
        for(std::size_t i = 0, total = segments_.size(); i < total; ++i) {
            segments_[i]->add(pose_, path_raw);


        }

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

    nav_msgs::Path plan (const geometry_msgs::PoseStamped &goal,
                         const lib_path::Pose2d& from_world, const lib_path::Pose2d& to_world,
                         const lib_path::Pose2d& from_map, const lib_path::Pose2d& to_map) {

        throw std::logic_error("should not be called");
    }

    void addCurve(double angle, double radius)
    {
        add(boost::shared_ptr<Segment>(new Curve(angle, radius, resolution_)));
    }

    void addStraight(double length)
    {
        add(boost::shared_ptr<Segment>(new Straight(length, resolution_)));
    }

    void add(const boost::shared_ptr<Segment>& segment)
    {
        segments_.push_back(segment);
    }

private:
    double resolution_;
    std::vector< boost::shared_ptr<Segment> > segments_;

    nav_msgs::Path path_;
    tf::Transform pose_;
    ros::Publisher posearray_pub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_planner");

    StaticPathPlanner planner;

    {
        ros::NodeHandle nh("~");
        XmlRpc::XmlRpcValue segment_array;
        nh.param("segments", segment_array, segment_array);

        if(segment_array.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_FATAL_STREAM("segments type is not array: " << segment_array.toXml());
            std::abort();
        }

        for(int i =0; i < segment_array.size(); i++)
        {
            XmlRpc::XmlRpcValue& segment = segment_array[i];
            if(segment.getType() != XmlRpc::XmlRpcValue::TypeArray) {
                ROS_FATAL_STREAM("segment type is not array: " << segment.toXml());
                std::abort();
            }

            if(segment.size() == 1) {
                planner.addStraight(segment[0]);

            } else if(segment.size() == 2) {
                planner.addCurve(segment[0], segment[1]);

            } else {
                ROS_FATAL_STREAM("segment with " << segment.size() << " elements is not defined");
                std::abort();
            }
        }
    }

    ros::WallRate r(2);
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }
}
