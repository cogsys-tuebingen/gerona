

#include "utils_pose_estimator.h"
#include "tf/transform_broadcaster.h"



UtilsPoseEstimator::UtilsPoseEstimator()
{

}



void UtilsPoseEstimator::Initialize(ros::NodeHandle &nodeP)
{
    nodeP.param("mapFrame", mapFrame_,std::string("map"));
    nodeP.param("baseLinkFrame", baseFrame_,std::string("base_link"));
    nodeP.param("localMapFrame", localMapFrame_,std::string("local_map"));
    int mapResolution;
    double mapSize;
    nodeP.param("mapResolution", mapResolution,1024);
    nodeP.param("mapSize", mapSize,16.0);

    double pixelResolution = (double)mapResolution / mapSize;

    double mapScale;
    double mapOffset;
    nodeP.param("mapScale", mapScale,1000.0);
    nodeP.param("mapOffset", mapOffset,10000.0);



    std::string robotConfig;
    nodeP.param("robot_config_file", robotConfig,std::string(""));
    nodeP.param("usePoseEstimator", usePoseEstimator_,false);

    hasPoseEstimator_ = false;
    if (usePoseEstimator_ && robotConfig.length() > 2)
    {
        ModelBasedPlannerConfig peConfig;

        if (peConfig.ReadRobotDescription(robotConfig))
        {
            hasPoseEstimator_ = true;
            peConfig.procConfig_.heightScale = mapScale;
            peConfig.procConfig_.mapBaseHeight = mapOffset;
            peConfig.procConfig_.wheelGroundLevel = mapOffset*2.0;
            peConfig.procConfig_.maxHeight = mapOffset*3.0;
            peConfig.procConfig_.pixelSize = 1.0/pixelResolution;
            peConfig.procConfig_.imagePosBLMinX = -mapSize /2;
            peConfig.procConfig_.imagePosBLMinY = -mapSize /2;
            peConfig.procConfig_.validThresholdFactor = 0.95;
            peConfig.Setup();
            poseEstimator_.Setup(peConfig);

        }



    }

    mapCounter_ = -2;
    nodeP.param("mapSkipVal", mapSkipVal_,120);


}

bool UtilsPoseEstimator::GetTransform(ros::Time time,std::string targetFrame, std::string sourceFrame, tf::StampedTransform &trans)
{
    try{//Try to get the latest avaiable Transform
        tf_listener.lookupTransform(targetFrame, sourceFrame, time, trans);
    }catch(tf::TransformException ex){//if not available, then wait
        (void) ex;
        if(!tf_listener.waitForTransform(targetFrame, sourceFrame, time, ros::Duration(0.05))){
            ROS_WARN_STREAM_THROTTLE(0.5,"DE_Localmap: cannot lookup transform from: " << targetFrame << " to " << sourceFrame);
            return false;
        }
        tf_listener.lookupTransform(targetFrame, sourceFrame, time, trans);
    }

    return true;


}

void UtilsPoseEstimator::GetEstimate(tf::StampedTransform &robotPose)
{
    tf::StampedTransform base2map;

    bool lookUpOk = GetTransform(ros::Time(0),mapFrame_, baseFrame_, base2map);
    if (!lookUpOk) {
        ROS_ERROR_STREAM("Error looking up Base to Map transform: " << mapFrame_ << " to " << baseFrame_);

        return;

    }

    cv::Point2f robotPos;
    robotPos.x = (base2map.getOrigin().x());
    robotPos.y = (base2map.getOrigin().y());

    tf::Matrix3x3 rotMat(base2map.getRotation());
    double bRoll,bPitch,bYaw;
    rotMat.getRPY(bRoll,bPitch,bYaw);


    PoseEvalResults results;
    cv::Point3f rPose(robotPos.x,robotPos.y,bYaw);
    results.pose = poseEstimator_.PoseToImgPose(rPose, curMapPos_);

    poseEstimator_.Evaluate(results);


    geometry_msgs::PoseStamped newPose;


    newPose.pose.position.x = robotPos.x;
    newPose.pose.position.y = robotPos.y;
    newPose.pose.position.z = (results.z1+results.z2)/2.0;


    tf::Quaternion q1 = tf::createQuaternionFromRPY(results.r1,results.p1,bYaw);
    tf::Quaternion q2 = tf::createQuaternionFromRPY(results.r2,results.p2,bYaw);

    tf::Quaternion qres = q1.slerp(q2,0.5);

    tf::quaternionTFToMsg(qres,newPose.pose.orientation);

    newPose.header.stamp = ros::Time::now();
    newPose.header.frame_id = baseFrame_;

    tf::Transform tfPose;
    tf::poseMsgToTF(newPose.pose,tfPose);


    tf::Transform correction;
    correction.setRotation(tf::createQuaternionFromRPY(results.r1,results.p1,0));
    correction.setOrigin(tf::Vector3(0,0,(results.z1+results.z2)/2.0));

    tf::Transform rTrans = base2map*correction;


    static tf::TransformBroadcaster br;

    //br.sendTransform(tf::StampedTransform(correction, ros::Time::now(), baseFrame_, "testBaseLink"));
    br.sendTransform(tf::StampedTransform(rTrans, ros::Time::now(), mapFrame_, "testBaseLink"));

    robotPose = tf::StampedTransform(rTrans, ros::Time::now(), mapFrame_, baseFrame_);

    /*
    robotPose.stamp_ =  ros::Time::now();;
    robotPose.frame_id_ = mapFrame_;
    robotPose.child_frame_id_ = baseFrame_;
    robotPose.setOrigin(newPose.pose.position);
    robotPose.setRotation(newPose.pose.orientation);

    static tf::TransformBroadcaster br;

    br.sendTransform(robotPose);
    */


}


void UtilsPoseEstimator::UpdateLocalMap(const cv::Mat &dem, const cv::Point2f &mapOrigin)
{
    mapCounter_++;
    if (mapCounter_%mapSkipVal_ != 0) return;
    curMapPos_ = mapOrigin;

    cv::Mat nDem;// = dem;

    if (dem.type() != CV_16U)
    {
        dem.convertTo(nDem,CV_16U);
    }
    else
    {
        nDem = dem;
    }

    poseEstimator_.SetDem(nDem);

}
