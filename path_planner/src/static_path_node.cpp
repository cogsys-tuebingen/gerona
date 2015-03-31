/// COMPONENT
#include "planner_node.h"

/// SYSTEM
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
        do {
            double delta_theta = sign * std::min(std::abs(angle) - span, resolution);
            span += std::abs(delta_theta);

            double theta = tf::getYaw(pose.getRotation());
            tf::Vector3 delta_pos(radius * (-std::sin(theta) + std::sin(theta + delta_theta)),
                                  radius * (std::cos(theta) - std::cos(theta + delta_theta)),
                                  0);

            pose.setOrigin(pose.getOrigin() + sign*delta_pos);
            pose.setRotation(tf::createQuaternionFromYaw(theta + delta_theta));

            path.poses.push_back(tf2pose(pose));

        } while(span < std::abs(angle));
    }
};

struct Straight : public Segment
{
    double length;

    Straight(double length, double resolution)
        : Segment(resolution), length(length)
    {}

    void add(tf::Transform& pose, nav_msgs::Path& path)
    {
        double distance = 0;
        do {
            double step = std::min(length - distance, resolution);
            tf::Transform delta(tf::createIdentityQuaternion(), tf::Vector3(step, 0, 0));
            distance += step;

            pose = pose * delta;

            path.poses.push_back(tf2pose(pose));

        } while(distance < length);
    }
};

struct StaticPathPlanner : public Planner
{
    StaticPathPlanner()
    {
        nh.param("resolution", resolution, 0.1);
    }


    void execute(const path_msgs::PlanPathGoalConstPtr &goal)
    {
        nav_msgs::Path path_raw = nav_msgs::Path();

        tf::poseMsgToTF(goal->goal.pose, pose);
//        pose = tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(0,0,0));

        path_raw.header.frame_id = "map";
        path_raw.header.stamp = ros::Time::now();

        for(std::size_t i = 0, total = segments.size(); i < total; ++i) {
            segments[i]->add(pose, path_raw);
        }

        path = postprocess(path_raw);
        publish(path, path_raw);

        feedback(path_msgs::PlanPathFeedback::STATUS_DONE);

        path_msgs::PlanPathResult success;
        success.path = path;
        server_.setSucceeded(success);
    }

    nav_msgs::Path plan (const geometry_msgs::PoseStamped &goal,
                         const lib_path::Pose2d& from_world, const lib_path::Pose2d& to_world,
                         const lib_path::Pose2d& from_map, const lib_path::Pose2d& to_map) {

        throw std::logic_error("should not be called");
    }

    void addCurve(double angle, double radius)
    {
        add(boost::shared_ptr<Segment>(new Curve(angle, radius, resolution)));
    }

    void addStraight(double length)
    {
        add(boost::shared_ptr<Segment>(new Straight(length, resolution)));
    }

    void add(const boost::shared_ptr<Segment>& segment)
    {
        segments.push_back(segment);
    }

private:
    double resolution;
    std::vector< boost::shared_ptr<Segment> > segments;

    nav_msgs::Path path;
    tf::Transform pose;
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
