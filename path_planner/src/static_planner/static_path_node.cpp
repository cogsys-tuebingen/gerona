/// COMPONENT
#include "../planner_node.h"

/// SYSTEM
#include <math.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <memory>

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

    virtual bool isForward() const = 0;

    virtual void add(tf::Transform& pose, path_msgs::PathSequence& path) = 0;
};

struct Curve : public Segment
{
    double angle;
    double radius;

    Curve(double angle, double radius, double resolution)
        : Segment(resolution), angle(angle), radius(radius)
    {}

    virtual bool isForward() const override
    {
        return radius >= 0.0;
    }

    void add(tf::Transform& pose, path_msgs::PathSequence& path_seq)
    {
        double span = 0;
        double sign = angle > 0 ? 1.0 : -1.0;
        double sign_delta_theta = radius>=0?1.0 : -1.0;

        path_msgs::DirectionalPath* path = &path_seq.paths.back();

        do {
            double delta_theta = sign * std::min(std::abs(angle) - span, resolution);
            span += std::abs(delta_theta);

            double theta = tf::getYaw(pose.getRotation());
            tf::Vector3 delta_pos(radius * (-std::sin(theta) + std::sin(theta + delta_theta)),
                                  radius * (std::cos(theta) - std::cos(theta + delta_theta)),
                                  0);

            pose.setOrigin(pose.getOrigin() + sign*delta_pos);
            pose.setRotation(tf::createQuaternionFromYaw(theta + sign_delta_theta*delta_theta));

            path->poses.push_back(tf2pose(pose));

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

    virtual bool isForward() const override
    {
        return dir_ > 0;
    }

    void add(tf::Transform& pose, path_msgs::PathSequence& path_seq)
    {
        path_msgs::DirectionalPath* path = &path_seq.paths.back();

        int steps = std::ceil(length_ / resolution);
        double exact_step_length = length_ / steps;
        tf::Transform delta(tf::createIdentityQuaternion(), tf::Vector3(dir_*exact_step_length, 0, 0));

        for(int step = 0; step < steps; ++step) {
            pose = pose * delta;
            path->poses.push_back(tf2pose(pose));
        }
    }
};

struct StaticPathPlanner : public Planner
{
    StaticPathPlanner()
    {
        posearray_pub_ = nh_priv.advertise<geometry_msgs::PoseArray>("static_poses",1000);
        nh_priv.param("resolution", resolution_, 0.1);
    }

    virtual bool supportsGoalType(int /*type*/) const override
    {
        return true;
    }

    void execute(const path_msgs::PlanPathGoalConstPtr &goal)
    {
        if(!generatePath()) {
            path_msgs::PlanPathResult failure;
            server_.setSucceeded(failure);
            return;
        }

        path_msgs::PathSequence path_raw = path_msgs::PathSequence();

        tf::poseMsgToTF(goal->goal.pose.pose, pose_);
        //        pose = tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(0,0,0));

        path_raw.header.frame_id = world_frame_;
        path_raw.header.stamp = ros::Time::now();
        geometry_msgs::PoseArray poses;

        int last_dir = 0;
        for(std::size_t i = 0, total = segments_.size(); i < total; ++i) {
            std::shared_ptr<Segment> s = segments_[i];
            int dir = s->isForward() ? 1 : -1;
            if(dir != last_dir) {
                path_raw.paths.emplace_back();
                path_raw.paths.back().forward = s->isForward();
            }
            last_dir = dir;

            s->add(pose_, path_raw);

        }

        path_ = postprocess(path_raw);

        publish(path_, path_raw);
        for(const path_msgs::DirectionalPath& path : path_raw.paths) {
            for (const geometry_msgs::PoseStamped& spose : path.poses) {
                poses.poses.push_back(spose.pose);
            }
        }
        poses.header.frame_id = world_frame_;
        posearray_pub_.publish(poses);
        feedback(path_msgs::PlanPathFeedback::STATUS_DONE);

        path_msgs::PlanPathResult success;
        success.path = path_;
        server_.setSucceeded(success);
    }

    path_msgs::PathSequence plan (const path_msgs::PlanPathGoal &goal,
                                  const lib_path::Pose2d& from_world, const lib_path::Pose2d& to_world,
                                  const lib_path::Pose2d& from_map, const lib_path::Pose2d& to_map) {

        throw std::logic_error("should not be called");
    }

    void addCurve(double angle, double radius)
    {
        add(std::make_shared<Curve>(angle, radius, resolution_));
    }

    void addStraight(double length)
    {
        add(std::make_shared<Straight>(length, resolution_));
    }

    void add(const std::shared_ptr<Segment>& segment)
    {
        segments_.push_back(segment);
    }

private:

    bool generatePath()
    {
        ros::NodeHandle nh("~");
        XmlRpc::XmlRpcValue segment_array;
        nh.param("segments", segment_array, segment_array);

        segments_.clear();

        if(segment_array.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_FATAL_STREAM("segments type is not array: " << segment_array.toXml());
            return false;
        }

        for(int i =0; i < segment_array.size(); i++)
        {
            XmlRpc::XmlRpcValue& segment = segment_array[i];
            if(segment.getType() != XmlRpc::XmlRpcValue::TypeArray) {
                ROS_FATAL_STREAM("segment type is not array: " << segment.toXml());
                return false;
            }

            if(segment.size() == 1) {
                addStraight(segment[0]);

            } else if(segment.size() == 2) {
                addCurve(segment[0], segment[1]);

            } else {
                ROS_FATAL_STREAM("segment with " << segment.size() << " elements is not defined");
                return false;
            }
        }

        return true;
    }

private:
    double resolution_;
    std::vector< std::shared_ptr<Segment> > segments_;

    path_msgs::PathSequence path_;
    tf::Transform pose_;
    ros::Publisher posearray_pub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_planner");

    StaticPathPlanner planner;

    ros::WallRate r(2);
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }
}
