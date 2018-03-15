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


struct GoalPathPlanner : public Planner
{
    GoalPathPlanner()
    {
        posearray_pub_ = nh_priv.advertise<geometry_msgs::PoseArray>("static_poses",1000);
        nh_priv.param("resolution", resolution_, 0.5);
    }

    virtual bool supportsGoalType(int /*type*/) const override
    {
        return true;
    }


    float getDistance(cv::Point2f p1, cv::Point2f p2)
    {
        cv::Point2f dif = p2-p1;
        return sqrt(dif.dot(dif));

    }

    void execute(const path_msgs::PlanPathGoalConstPtr &goal)
    {
        geometry_msgs::PoseStamped start = lookupPose();


        cv::Point2f startPos(start.pose.position.x,start.pose.position.y),
                endPos(goal->goal.pose.pose.position.x,goal->goal.pose.pose.position.y);


        path_msgs::PathSequence path_raw = path_msgs::PathSequence();

        path_msgs::DirectionalPath path = path_msgs::DirectionalPath();

        tf::poseMsgToTF(goal->goal.pose.pose, pose_);
        //        pose = tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(0,0,0));

        path_raw.header.frame_id = world_frame_;
        path_raw.header.stamp = ros::Time::now();
        geometry_msgs::PoseArray poses;


        cv::Point2f dir = endPos-startPos;

        cv::Point2f dirN = dir *(1.0/sqrt(dir.dot(dir)));

        cv::Point2f curPos = startPos;

        float orientation = atan2(dir.y,dir.x);

        tf::Transform pose;

        while (getDistance(curPos,endPos) > resolution_)
        {

            pose.setOrigin(tf::Vector3(curPos.x,curPos.y,0));
            pose.setRotation(tf::createQuaternionFromYaw(orientation));

            path.poses.push_back(tf2pose(pose));

            curPos = curPos+dirN*resolution_;
        }


        pose.setOrigin(tf::Vector3(endPos.x,endPos.y,0));
        pose.setRotation(tf::createQuaternionFromYaw(orientation));

        path.poses.push_back(tf2pose(pose));


        path_raw.paths.push_back(path);

        /*
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
        */



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





private:
    double resolution_;

    path_msgs::PathSequence path_;
    tf::Transform pose_;
    ros::Publisher posearray_pub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_planner");

    GoalPathPlanner planner;

    ros::WallRate r(2);
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }
}
