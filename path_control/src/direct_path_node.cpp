#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <path_msgs/NavigateToGoalAction.h>
#include <path_msgs/PathSequence.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/String.h>
#include <string>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
//#include <path_msgs/NavigateToGoalAction.h>
#include <path_msgs/FollowPathAction.h>

using namespace path_msgs;

/**
 * @brief Simple high level control dummy for testing.
 *
 * This dummy subscribes for the goals set manually in rviz. The goals are then wrapped in an action and send to
 * path_control.
 */

class DirectPath
{
public:
    DirectPath(ros::NodeHandle &nh):
        nh_(nh),
        pnh_("~"),
        follow_path_client_("follow_path")
    {


        //srand(ros::Time::now().toNSec());

        // topic for path
        path_sub_ = nh.subscribe<path_msgs::PathSequence>("direct_path", 1, &DirectPath::pathCb, this);

        // topic for goal position
        goal_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("direct_goal", 0, &DirectPath::goalCb, this);


        follow_path_client_.waitForServer();

        speech_pub_ = nh.advertise<std_msgs::String>("speech", 0);

        std::string default_world_frame = nh_.param("gerona/world_frame", std::string("map"));
        pnh_.param("world_frame", world_frame_, default_world_frame);
        std::string default_base_frame = "/base_link";
        pnh_.param("base_frame", base_frame_, default_base_frame);

        // target speed
        pnh_.param("target_speed", target_speed_, 1.0);

        // default init mode
        //pnh_.param("init_mode", init_mode_, (int)FollowerOptions::INIT_MODE_CONTINUE);
        pnh_.param("init_mode", init_mode_, (int)FollowerOptions::INIT_MODE_STOP);

        // num interpolations of path points
        pnh_.param("num_interpolate_poses", num_interpolate_poses_, 10);



        // failure mode; possible: ABORT, REPLAN
        std::string failure_mode = "ABORT";
        pnh_.param("failure_mode", failure_mode, failure_mode);

        std::transform(failure_mode.begin(), failure_mode.end(), failure_mode.begin(), ::toupper);

        if(failure_mode == "ABORT") {
            failure_mode_ = path_msgs::NavigateToGoalGoal::FAILURE_MODE_ABORT;
        } else if(failure_mode == "REPLAN") {
            failure_mode_ = path_msgs::NavigateToGoalGoal::FAILURE_MODE_REPLAN;
        } else {
            ROS_WARN_STREAM("failure mode " << failure_mode << " is unknown. Defaulting to ABORT");
            failure_mode_ = path_msgs::NavigateToGoalGoal::FAILURE_MODE_ABORT;
        }



        ROS_INFO_STREAM("listening for goal @ " << path_sub_.getTopic());
        ROS_INFO_STREAM("failure mode is " << failure_mode);

        ROS_INFO("Client is set up");
    }


private:

    typedef actionlib::SimpleActionClient<path_msgs::FollowPathAction> FollowPathClient;

    //! Action client to communicate with the path_follower package.
    FollowPathClient follow_path_client_;

    //! Final state of the last finished follow_path action.
    actionlib::SimpleClientGoalState::StateEnum follow_path_final_state_;
    //! Result of the last finished follow_path action
    path_msgs::FollowPathResultConstPtr follow_path_result_;
    //! False if follow_path action is currently running, otherwise true.
    bool follow_path_done_;

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber path_sub_;
    ros::Subscriber goal_sub_;
    ros::Publisher speech_pub_;

    tf::TransformListener tfl_;

    std::string world_frame_;
    std::string base_frame_;

    double target_speed_;
    int failure_mode_;
    int init_mode_;
    int num_interpolate_poses_;

    bool GetTransform(std::string targetFrame, std::string sourceFrame,ros::Time time, tf::StampedTransform &trans)
    {
        try{//Try to get the latest avaiable Transform
            tfl_.lookupTransform(targetFrame, sourceFrame, time, trans);
        }catch(tf::TransformException ex){//if not available, then wait
            (void) ex;
            if(!tfl_.waitForTransform(targetFrame, sourceFrame, time, ros::Duration(0.5))){
                ROS_WARN_STREAM_THROTTLE(1, "cannot lookup transform from :" << targetFrame << " to " << sourceFrame);
                return false;
            }
            tfl_.lookupTransform(targetFrame, sourceFrame, time, trans);
        }

        return true;


    }

    bool TransformPose(const geometry_msgs::PoseStampedConstPtr &pose, std::string targetFrame, geometry_msgs::PoseStamped &result )
    {
        try{//Try to get the latest avaiable Transform
            tfl_.transformPose(targetFrame,*pose,result);
        }catch(tf::TransformException ex){//if not available, then wait
            (void) ex;
            if(!tfl_.waitForTransform(targetFrame, pose->header.frame_id, pose->header.stamp, ros::Duration(0.5))){
                ROS_WARN_STREAM_THROTTLE(1, "cannot lookup transform from: " << targetFrame << " to " << pose->header.frame_id);
                return false;
            }
            tfl_.transformPose(targetFrame,*pose,result);
        }

        return true;


    }

    bool TransformPath(const path_msgs::PathSequenceConstPtr &path, std::string targetFrame, path_msgs::PathSequence &result )
    {
        result.header.frame_id = targetFrame;
        result.header.stamp = path->header.stamp;
        result.paths.clear();
        try{//Try to get the latest avaiable Transform
            for (int pl = 0; pl < path->paths.size();++pl)
            {
                path_msgs::DirectionalPath dpath;

                dpath.forward = true;
                for (int dl = 0; dl < path->paths[pl].poses.size();++dl)
                {
                    geometry_msgs::PoseStamped resPose;
                    tfl_.transformPose(targetFrame,path->paths[pl].poses[dl],resPose);
                    dpath.poses.push_back(resPose);
                }
                result.paths.push_back(dpath);
            }

        }catch(tf::TransformException ex){//if not available, then wait
            (void) ex;
            if(!tfl_.waitForTransform(targetFrame, path->header.frame_id, path->header.stamp, ros::Duration(1.0))){
                ROS_WARN_STREAM_THROTTLE(1, "cannot lookup transform from :" << targetFrame << " to " << path->header.frame_id);
                return false;
            }
            for (int pl = 0; pl < path->paths.size();++pl)
            {
                path_msgs::DirectionalPath dpath;

                dpath.forward = true;
                for (int dl = 0; dl < path->paths[pl].poses.size();++dl)
                {
                    geometry_msgs::PoseStamped resPose;
                    tfl_.transformPose(targetFrame,path->paths[pl].poses[dl],resPose);
                    dpath.poses.push_back(resPose);
                }
                result.paths.push_back(dpath);
            }
        }

        return true;


    }


    geometry_msgs::PoseStamped GetPoseFromTransform(tf::StampedTransform tf) {
        //clumsy conversions--points, vectors and quaternions are different data types in tf vs geometry_msgs
        geometry_msgs::PoseStamped stPose;
        geometry_msgs::Quaternion quat;  //geometry_msgs object for quaternion
        tf::Quaternion tfQuat; // tf library object for quaternion
        tfQuat = tf.getRotation(); // member fnc to extract the quaternion from a transform
        quat.x = tfQuat.x(); // copy the data from tf-style quaternion to geometry_msgs-style quaternion
        quat.y = tfQuat.y();
        quat.z = tfQuat.z();
        quat.w = tfQuat.w();
        stPose.pose.orientation = quat; //set the orientation of our PoseStamped object from result

        // now do the same for the origin--equivalently, vector from parent to child frame
        tf::Vector3 tfVec;  //tf-library type
        geometry_msgs::Point pt; //equivalent geometry_msgs type
        tfVec = tf.getOrigin(); // extract the vector from parent to child from transform
        pt.x = tfVec.getX(); //copy the components into geometry_msgs type
        pt.y = tfVec.getY();
        pt.z = tfVec.getZ();
        stPose.pose.position= pt; //and use this compatible type to set the position of the PoseStamped
        stPose.header.frame_id = tf.frame_id_; //the pose is expressed w/rt this reference frame
        stPose.header.stamp = tf.stamp_; // preserve the time stamp of the original transform
        return stPose;
    }

    bool TestVectorLength(tf::Point start, tf::Point end)
    {
        const double epsilon = 0.01;
        return (sqrt(start.dot(end)) > epsilon);

    }

    bool TestVectorLength(tf::Vector3 dir)
    {
        const double epsilon = 0.01;
        return (sqrt(dir.dot(dir)) > epsilon);

    }

    void goalCb(const geometry_msgs::PoseStampedConstPtr &pose)
    {
        ROS_INFO("Send goal...");

        //ros::Time pTime = pose->header.stamp;
        //ros::Time zTime = ros::Time(0);

        if (pose->header.frame_id == "")
        {
            ROS_WARN("Path frame id is empty. Canceling all goals!");
            follow_path_client_.cancelAllGoals();

        }


        geometry_msgs::PoseStamped transGoal;



        boost::shared_ptr<geometry_msgs::PoseStamped> posePtr = boost::make_shared<geometry_msgs::PoseStamped>();

        //geometry_msgs::PoseStamped pose2 = pose;

        *posePtr.get() = *pose.get();

        posePtr->header.stamp =  ros::Time(0);

        bool hasTransform = TransformPose(posePtr,world_frame_,transGoal);

        if (!hasTransform) return;



        tf::StampedTransform map2base;
        //hasTransform = GetTransform(base_frame_,world_frame_,pose->header.stamp,map2base);
        hasTransform = GetTransform(world_frame_,base_frame_,pose->header.stamp,map2base);
        //hasTransform = GetTransform(base_frame_,world_frame_,zTime,map2base);
        if (!hasTransform) return;

        geometry_msgs::PoseStamped basePose = GetPoseFromTransform(map2base);

        path_msgs::DirectionalPath dpath;

        dpath.forward = true;

        dpath.poses.push_back(basePose);

        tf::Point start;
        tf::pointMsgToTF(basePose.pose.position,start);
        tf::Point end;
        tf::pointMsgToTF(transGoal.pose.position,end);

        tf::Vector3 dir = end-start;

        if (!TestVectorLength(dir))
        {
            ROS_WARN("End too close to start, |end-start| < 0.01m !!");
            return;
        }

        for (int x = 1; x < num_interpolate_poses_-1;x++)
        {

            geometry_msgs::PoseStamped curPose = basePose;

            tf::Point curPoint = start+dir*((double)x / (double)num_interpolate_poses_ );

            tf::pointTFToMsg(curPoint,curPose.pose.position);
            dpath.poses.push_back(curPose);

        }

        dpath.poses.push_back(transGoal);

        dpath.header = transGoal.header;



        boost::shared_ptr<path_msgs::PathSequence> pathPtr = boost::make_shared<path_msgs::PathSequence>();

        pathPtr->paths.push_back(dpath);
        pathPtr->header = transGoal.header;



        pathCb(pathPtr);



    }

    void pathCb(const path_msgs::PathSequenceConstPtr &path)
    {
        ROS_INFO("Send Path...");
        follow_path_client_.waitForServer();

        if (path->header.frame_id == "")
        {
            ROS_WARN("Path frame id is empty. Canceling all goals!");
            follow_path_client_.cancelAllGoals();

        }

        if (path->paths.empty())
        {
            ROS_WARN("Received Empty Path. Canceling all goals!");
            follow_path_client_.cancelAllGoals();

        }

        //follow_path_client_.

        path_msgs::FollowPathGoal path_action_goal;

        if (path->header.frame_id != world_frame_)
        {
            TransformPath(path,world_frame_,path_action_goal.path);
        }
        else path_action_goal.path = *path;
        path_action_goal.follower_options.velocity = target_speed_;
        path_action_goal.follower_options.init_mode = init_mode_;

        follow_path_client_.sendGoal(path_action_goal,
                                     boost::bind(&DirectPath::followPathDoneCB, this, _1, _2),
                                     boost::bind(&DirectPath::followPathActiveCB, this),
                                     boost::bind(&DirectPath::followPathFeedbackCB, this, _1));

        //follow_path_client_.stopTrackingGoal();

    }



    void followPathDoneCB(const actionlib::SimpleClientGoalState &state,
                          const path_msgs::FollowPathResultConstPtr &result)
    {
        if (state == state.LOST) {
            ROS_ERROR("Lost connection to path follower.");
        }

        ROS_INFO("Path execution finished.\n---------------------");

        follow_path_final_state_ = state.state_;
        follow_path_result_ = result;
        follow_path_done_ = true;
    }

    void followPathActiveCB()
    {
        ROS_INFO("Path is now active.");
        // is there anything to do here?
    }

    void followPathFeedbackCB(const path_msgs::FollowPathFeedbackConstPtr &feedback)
    {
        //ROS_INFO_THROTTLE(1,"Driven distance: %g;  Distance to goal: %g", feedback->dist_driven, feedback->dist_goal);

        path_msgs::NavigateToGoalFeedback nav_feedback;

        switch(feedback->status) {
        case FollowPathFeedback::MOTION_STATUS_MOVING:
            nav_feedback.status = NavigateToGoalFeedback::STATUS_MOVING;
            break;

        case FollowPathFeedback::MOTION_STATUS_OBSTACLE:
            nav_feedback.status = NavigateToGoalFeedback::STATUS_OBSTACLE;
            break;

        case FollowPathFeedback::MOTION_STATUS_NO_LOCAL_PATH:
            nav_feedback.status = NavigateToGoalFeedback::STATUS_NO_LOCAL_PLAN;
            break;

        default:
            ROS_ERROR("Feedback: Unknown status code %d", feedback->status);
            break;
        }

        nav_feedback.obstacles_on_path = feedback->obstacles_on_path;

        //navigate_to_goal_server_.publishFeedback(nav_feedback);

        //publishGoalMessage();
    }


    void say(std::string text)
    {
        std_msgs::String str;
        str.data = text;
        speech_pub_.publish(str);
    }
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "direct_path_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    DirectPath dummy(nh);

    ros::spin();
    return 0;
}

