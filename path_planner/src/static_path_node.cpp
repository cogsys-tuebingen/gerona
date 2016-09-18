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
    virtual void add(tf::Transform& pose, path_msgs::PathSequence& path) = 0;
};

struct Curve : public Segment
{
    double angle;
    double radius;

    Curve(double angle, double radius, double resolution)
        : Segment(resolution), angle(angle), radius(radius)
    {}

    void add(tf::Transform& pose, path_msgs::PathSequence& path_seq)
    {
        double span = 0;
        double sign = angle > 0 ? 1.0 : -1.0;
        double sign_delta_theta = radius>=0?1.0 : -1.0;

        if(path_seq.paths.empty()) {
            path_seq.paths.emplace_back();
        }

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

    void add(tf::Transform& pose, path_msgs::PathSequence& path_seq)
    {
        if(path_seq.paths.empty()) {
            path_seq.paths.emplace_back();
        }

        path_msgs::DirectionalPath* path = &path_seq.paths.back();

        double distance = 0;
        do {
            double step = std::min(length_ - distance, resolution);
            tf::Transform delta(tf::createIdentityQuaternion(), tf::Vector3(dir_*step, 0, 0));
            distance += step;

            pose = pose * delta;

            path->poses.push_back(tf2pose(pose));

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

    virtual bool supportsGoalType(int /*type*/) const override
    {
        return true;
    }

    void execute(const path_msgs::PlanPathGoalConstPtr &goal)
    {
        path_msgs::PathSequence path_raw = path_msgs::PathSequence();

        tf::poseMsgToTF(goal->goal.pose.pose, pose_);
        //        pose = tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(0,0,0));

        path_raw.header.frame_id = "map";
        path_raw.header.stamp = ros::Time::now();
        geometry_msgs::PoseArray poses;
        for(std::size_t i = 0, total = segments_.size(); i < total; ++i) {
            segments_[i]->add(pose_, path_raw);


        }


        ///////////////////////////////////////////////////////////////////////
        /// This is just a hacked solution to plan a lemniscate
        //////////////////////////////////////////////////////////////////////
        /*double a = 3.0;

        path_raw.poses.clear();

        geometry_msgs::PoseStamped first_pose = goal->goal.pose;

        double N_lem = 900;
        double f = 10*M_PI / (double) (N_lem-1);

        for(std::size_t i = 0; i < N_lem-1; i++){

            double t = i*f;
            geometry_msgs::PoseStamped curr_pose;

            curr_pose.pose.position.x = a*sqrt(2)*std::cos(t)/(pow(std::sin(t),2) + 1) + first_pose.pose.position.x - a*sqrt(2);
            curr_pose.pose.position.y = a*sqrt(2)*std::cos(t)*std::sin(t)/(pow(std::sin(t),2) + 1) + first_pose.pose.position.y;

            path_raw.poses.push_back(curr_pose);
        }*/

        ///////////////////////////////////////////////////////////////////////

        path_ = postprocess(path_raw);

        publish(path_, path_raw);
        for(const path_msgs::DirectionalPath& path : path_raw.paths) {
            for (const geometry_msgs::PoseStamped& spose : path.poses) {
                poses.poses.push_back(spose.pose);
            }
        }
        poses.header.frame_id="/map";
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

    path_msgs::PathSequence path_;
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
