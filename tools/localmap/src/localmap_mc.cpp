
#include "localmap_mc.h"



LocalmapMC::LocalmapMC() :
    nodeG_(),
    nodeP_("~")
{


    nodeP_.param("numCameras", numCameras_,1);

    for (int i = 0; i < numCameras_;i++)
    {
        //scan_sub_front_ = node_.subscribe<sensor_msgs::LaserScan>("scan/front/filtered", 1, boost::bind(&ScanConverter::scanCallback, this, _1, false));
        std::ostringstream ossDepth;
        ossDepth << "/depth_image" << i;
        depthImageSubs_.push_back(std::move(nodeG_.subscribe<sensor_msgs::Image> (ossDepth.str().c_str(), numCameras_, boost::bind(&LocalmapMC::imageCallback, this, _1, i))));
        std::ostringstream ossCamInfo;
        ossCamInfo << "/camera_info" << i;
        cameraInfoSubs_.push_back(std::move(nodeG_.subscribe<sensor_msgs::CameraInfo>(ossCamInfo.str().c_str(), numCameras_, boost::bind(&LocalmapMC::ci_callback, this, _1, i))));

    }

    hasCamInfos_.resize(numCameras_,false);
    hasCam2Bases_.resize(numCameras_,false);
    camInfos_.resize(numCameras_);
    cam2Bases_.resize(numCameras_);


    //depthSub_ = nodeG_.subscribe ("/depth_image", 1, &LocalmapMC::imageCallback, this);
    //scan_sub_front_ = node_.subscribe<sensor_msgs::LaserScan>("scan/front/filtered", 1, boost::bind(&ScanConverter::scanCallback, this, _1, false));


    //cameraInfoSub_ = nodeG_.subscribe ("/camera_info", 1, &LocalmapMC::ci_callback, this);

    mapResetSub_ = nodeG_.subscribe ("/localmap_reset_trigger", 1, &LocalmapMC::mr_callback, this);

#ifdef ELEVATION_CLOUD_DEBUG
    imageCloud_pub_ =    nodeG_.advertise<sensor_msgs::PointCloud2>("/elevation_cloud",1);

#endif


    zImagePub_ =     nodeG_.advertise<sensor_msgs::Image>("/elevation_map",1);
    assignImagePub_ =     nodeG_.advertise<sensor_msgs::Image>("/assign_image",1);
    debugImagePub_ =     nodeG_.advertise<sensor_msgs::Image>("/debug_image",1);

    double tval;


    nodeP_.param("mapResolution", mapResolution_,1024);
    nodeP_.param("mapSize", tval,16.0);
    mapSize_ = tval;

    nodeP_.param("numBlocks", numBlocks_,8);
    blockStep_ = mapSize_ / (float)numBlocks_;


    pixelResolution_ = (double)mapResolution_ / mapSize_;

    proc_.pixelResolution_ = pixelResolution_;


    blockMap_.mapSize_ = mapSize_;
    blockMap_.mapResolution_ = mapResolution_;
    blockMap_.pixelResolution_ = pixelResolution_;
    blockMap_.numBlocks_ = numBlocks_;


    nodeP_.param("processMode", processMode_,(int)PM_INTERP);
    nodeP_.param("fuseMode", fuseMode_,(int)FM_OVERWRITE);
    nodeP_.param("output16U", output16U_,true);
    nodeP_.param("mapScale", mapScale_,1000.0);
    nodeP_.param("mapOffset", mapOffset_,10000.0);
    nodeP_.param("mapZeroLevel", mapZeroLevel_,0.0);
    nodeP_.param("mapNotVisibleLevel", mapNotVisibleLevel_,1000.0);

    nodeP_.param("transform2BaseLink", transform2BaseLink_,true);
    nodeP_.param("useLatestTransform", useLatestTransform_,0);

    nodeP_.param("removeLeftImageCols", removeLeftImageCols_,-1);

    nodeP_.param("removeWindowStartCol", removeWindowStartCol_,-1);
    nodeP_.param("removeWindowEndCol", removeWindowEndCol_,-1);
    nodeP_.param("removeWindowStartRow", removeWindowStartRow_,-1);
    nodeP_.param("removeWindowEndRow", removeWindowEndRow_,-1);


    mapZeroLevelf_ = mapZeroLevel_;

    nodeP_.param("minAssignValue", tval,0.1);
    proc_.minAssignValue_ = tval;

    nodeP_.param("minDepthThreshold", tval,0.4);
    //proc_.minDepthThreshold_ = tval;


    nodeP_.param("mapFrame", mapFrame_,std::string("map"));
    nodeP_.param("baseLinkFrame", baseFrame_,std::string("base_link"));
    nodeP_.param("localMapFrame", localMapFrame_,std::string("local_map"));

    nodeP_.param("transformWaitTime", transformWaitTime_,0.1);
    nodeP_.param("resetWaitTime", resetWaitTime_,0.3);


    nodeP_.param("postProcessType", postProcessType_,0);
    nodeP_.param("postProcessSize", postProcessSize_,3);

    std::vector<float> testPlaneNormal_default = {1.0,0,0};

    nodeP_.param<std::vector<float> >("testPlaneNormal", testPlaneNormal_, testPlaneNormal_default);

    cv::Point3f tn(testPlaneNormal_[0],testPlaneNormal_[1],testPlaneNormal_[2]);
    double length = sqrt(tn.dot(tn));
    tn = tn * (1.0/length);
    testPlaneNormal_[0] = tn.x;
    testPlaneNormal_[1] = tn.y;
    testPlaneNormal_[2] = tn.z;

    nodeP_.param("testPlaneDistance", testPlaneDistance_,0.5f);
    nodeP_.param("doTestPlane", proc_.testPlane_,false);


    numRegistered_ = 0;
    totalRegisterTime_ = 0;

    //hasCamInfo_ = false;
    //hasCam2Base_ = false;

    cAssign_ = cv::Mat(mapResolution_,mapResolution_,CV_32F);
    cZImg_ = cv::Mat(mapResolution_,mapResolution_,CV_32F);

    cZImg_.setTo(mapZeroLevel_);
    cAssign_.setTo(0);

    blockMap_.mapNotVisibleLevel_ = mapNotVisibleLevel_;
    blockMap_.mapBaseLevel_ = mapOffset_;
    blockMap_.Setup();
    //blockMap_.SetSafeBlocksTo();
    blockMap_.heightScale_ = mapScale_;

    initBlockMap_ = true;




}



void LocalmapMC::SetupMatrices(tf::Transform &transform)
{
    tf::Matrix3x3 rotMat(transform.getRotation());

    proc_.r11 = rotMat[0][0];
    proc_.r12 = rotMat[0][1];
    proc_.r13 = rotMat[0][2];
    proc_.r21 = rotMat[1][0];
    proc_.r22 = rotMat[1][1];
    proc_.r23 = rotMat[1][2];
    proc_.r31 = rotMat[2][0];
    proc_.r32 = rotMat[2][1];
    proc_.r33 = rotMat[2][2];

    tf::Vector3 nTrans = transform.getOrigin();

    proc_.t1 = nTrans.x();
    proc_.t2 = nTrans.y();
    proc_.t3 = nTrans.z();


}


bool LocalmapMC::GetTransform(ros::Time time,std::string targetFrame, std::string sourceFrame, tf::StampedTransform &trans)
{
    try{//Try to get the latest avaiable Transform
        tf_listener.lookupTransform(targetFrame, sourceFrame, time, trans);
    }catch(tf::TransformException ex){//if not available, then wait
        (void) ex;
        if(!tf_listener.waitForTransform(targetFrame, sourceFrame, time, ros::Duration(transformWaitTime_))){ //ros::Duration(0.05))){
            ROS_WARN_STREAM_THROTTLE(0.5,"LocalmapMC: cannot lookup transform from: " << targetFrame << " to " << sourceFrame);
            return false;
        }
        tf_listener.lookupTransform(targetFrame, sourceFrame, time, trans);
    }

    return true;


}

void LocalmapMC::ci_callback(const sensor_msgs::CameraInfoConstPtr& info, int idx)
{
    if (hasCamInfos_[idx]) return;
    if (info->P.at(0) == 0) return;
    camInfos_[idx] = *info;

    hasCamInfos_[idx] = true;


    ROS_INFO_STREAM("Received camera info!");

}

void LocalmapMC::mr_callback(const std_msgs::Int8ConstPtr& data)
{
    ros::Duration durWait(resetWaitTime_);
    durWait.sleep();
    blockMap_.SetMapTo(0);
    blockMap_.SetSafeAroundRobot();

    ROS_WARN_STREAM("Localmap: Resetting!");

}



cv::Point2f LocalmapMC::ConvertPoint(cv::Point2f &p)
{
    return cv::Point2f((p.x-proc_.minXVal_) * proc_.pixelResolution_,(p.y-proc_.minYVal_) * proc_.pixelResolution_);

}


void LocalmapMC::UpdateLocalMapOverwrite(cv::Mat &localMap, const cv::Mat & zImage, const cv::Mat &assignImage, const cv::Vec4i &minMax)
{
    const float *zImageP;
    float *localMapP;
    const float *assignP;

    float minVal = proc_.minAssignValue_;
    int xl,yl;
    float mapScaleF = mapScale_;
    float mapOffsetF = mapOffset_;

    for (yl = minMax[1]; yl < minMax[3];++yl)
    {
        zImageP = zImage.ptr<float>(yl);
        localMapP = localMap.ptr<float>(yl);
        assignP = assignImage.ptr<float>(yl);

        for (xl = minMax[0]; xl < minMax[2];++xl)
        {
            if (assignP[xl] >= minVal)
            {
                localMapP[xl] = (zImageP[xl]/(assignP[xl]))*mapScaleF+mapOffsetF;

            }

        }
    }

}

void LocalmapMC::UpdateLocalMapOverwriteMax(cv::Mat &localMap, const cv::Mat & zImage, const cv::Mat &assignImage, const cv::Vec4i &minMax)
{
    const float *zImageP;
    float *localMapP;
    const float *assignP;

    float minVal = proc_.minAssignValue_;
    int xl,yl;
    float mapScaleF = mapScale_;
    float mapOffsetF = mapOffset_;

    for (yl = minMax[1]; yl < minMax[3];++yl)
    {
        zImageP = zImage.ptr<float>(yl);
        localMapP = localMap.ptr<float>(yl);
        assignP = assignImage.ptr<float>(yl);

        for (xl = minMax[0]; xl < minMax[2];++xl)
        {
            if (assignP[xl] >= minVal)
            {
                localMapP[xl] = (zImageP[xl])*mapScaleF+mapOffsetF;

            }

        }
    }

}


void LocalmapMC::UpdateLocalMapTemporal(cv::Mat &localMap, cv::Mat &localTempMap, const cv::Mat & zImage, const cv::Mat &assignImage, const cv::Vec4i &minMax, const cv::Point3f &planeP, const cv::Point3f &planeN)
{
    const float *zImageP;
    float *localMapP;
    const float *assignP;

    float minVal = proc_.minAssignValue_;
    int xl,yl;
    float mapScaleF = mapScale_;
    float mapOffsetF = mapOffset_;

    //float zVal = 0
    for (yl = minMax[1]; yl < minMax[3];++yl)
    {
        zImageP = zImage.ptr<float>(yl);
        localMapP = localTempMap.ptr<float>(yl);
        assignP = assignImage.ptr<float>(yl);

        for (xl = minMax[0]; xl < minMax[2];++xl)
        {
            if (assignP[xl] >= minVal)
            {
                localMapP[xl] = (zImageP[xl]/(assignP[xl]))*mapScaleF+mapOffsetF;;

            }

        }
    }

    float px,py,pd;

    float tppx,tppy,tppz;
    proc_.ToMapCoord(planeP.x,planeP.y,planeP.z,tppx,tppy,tppz);

    cv::Point3f tPlaneP(tppx,tppy,0);

    cv::Point3f tnorm = planeN * (1.0f/(std::sqrt(planeN.dot(planeN))));

    pd = -tPlaneP.dot(tnorm);
    px = tnorm.x;
    py = tnorm.y;

    float x,y;
    //cv::Mat tempAssign;
    //float *tempAssignP;

    //assignImage.copyTo(tempAssign);

    /*
    for (yl = minMax[1]; yl < minMax[3];++yl)
    {
        //zImageP = zImage.ptr<float>(yl);
        //localMapP = localMap.ptr<float>(yl);
        tempAssignP = tempAssign.ptr<float>(yl);

        for (xl = minMax[0]; xl < minMax[2];++xl)
        {
            x = (float)xl;
            y = (float)yl;
            if (x*px+y*py+pd < 0)
            {
                tempAssignP[xl] = -1.0f;
            }
        }

    }
    */

    for (yl = minMax[1]; yl < minMax[3];++yl)
    {
        zImageP = zImage.ptr<float>(yl);
        localMapP = localMap.ptr<float>(yl);
        assignP = assignImage.ptr<float>(yl);

        for (xl = minMax[0]; xl < minMax[2];++xl)
        {
            x = (float)xl;
            y = (float)yl;

            if (assignP[xl] >= minVal && x*px+y*py+pd > 0)
            {
                localMapP[xl] = (zImageP[xl]/(assignP[xl]))*mapScaleF+mapOffsetF;

            }

        }
    }

}


void LocalmapMC::UpdateLocalMapMax(cv::Mat &localMap, const cv::Mat & zImage, const cv::Mat &assignImage, const cv::Vec4i &minMax)
{
    const float *zImageP;
    float *localMapP;
    const float *assignP;

    float minVal = proc_.minAssignValue_;
    int xl,yl;
    float mapScaleF = mapScale_;
    float mapOffsetF = mapOffset_;

    //float zVal = 0
    for (yl = minMax[1]; yl < minMax[3];++yl)
    {
        zImageP = zImage.ptr<float>(yl);
        localMapP = localMap.ptr<float>(yl);
        assignP = assignImage.ptr<float>(yl);

        for (xl = minMax[0]; xl < minMax[2];++xl)
        {
            if (assignP[xl] >= minVal)
            {
                const float nval = (zImageP[xl]/(assignP[xl]))*mapScaleF+mapOffsetF;
                if (nval > localMapP[xl]) localMapP[xl] = nval;

            }

        }
    }

}




void LocalmapMC::imageCallback(const sensor_msgs::ImageConstPtr& depth, int idx)
{


    if (!hasCamInfos_[idx]) return;
    sensor_msgs::CameraInfo camInfo = camInfos_[idx];
    proc_.SetupCam(camInfo.P[0],camInfo.P[5],camInfo.P[2],camInfo.P[6]);

    std::string cameraFrame = depth->header.frame_id;
    ros::Time timeStamp = depth->header.stamp;
    if (useLatestTransform_ == 1) timeStamp = ros::Time::now();
    if (useLatestTransform_ == 2) timeStamp = ros::Time(0);

    if (!hasCam2Bases_[idx])
    {

        if (!GetTransform(ros::Time(0),baseFrame_, cameraFrame, cam2Bases_[idx])) {
            ROS_ERROR_STREAM("Error looking up Camera to Base transform: " << cameraFrame << " to " << baseFrame_);

            return;

        }
        hasCam2Bases_[idx] = true;
    }


    tf::StampedTransform base2map;
    bool lookUpOk = GetTransform(timeStamp,mapFrame_, baseFrame_, base2map);
    if (!lookUpOk) {
        ROS_ERROR_STREAM_THROTTLE(0.5,"Error looking up Base to Map transform: " << baseFrame_ << " to " << mapFrame_ << " Time: " << timeStamp);

        return;

    }


    {
        cv::Point3f pos(base2map.getOrigin().x(),base2map.getOrigin().y(),base2map.getOrigin().z());

        tf::Vector3 upVec(0,0,1.0);

        tf::Transform tempTrans;

        tempTrans.setIdentity();

        tempTrans.setRotation(base2map.getRotation());

        tf::Vector3 tf_normal = tempTrans*upVec;


        cv::Point3f normal(tf_normal.x(),tf_normal.y(),tf_normal.z());

        blockMap_.SetPose(normal,pos);


    }

    /*
    tf::Vector3 planePointBase(testPlaneNormal_[0]*testPlaneDistance_,testPlaneNormal_[1]*testPlaneDistance_,testPlaneNormal_[2]*testPlaneDistance_);
    tf::Vector3 planeNormalBase(testPlaneNormal_[0],testPlaneNormal_[1],testPlaneNormal_[2]);

    tf::Vector3 planePointMap = base2map*planePointBase;
    tf::Transform base2mapRot;
    base2mapRot.setIdentity();
    base2mapRot.setRotation(base2map.getRotation());
    tf::Vector3 planeNormalMap = base2mapRot*planeNormalBase;
    proc_.SetPlane(cv::Point3f(planePointMap.x(),planePointMap.y(),planePointMap.z()),cv::Point3f(planeNormalMap.x(),planeNormalMap.y(),planeNormalMap.z()));

    ROS_INFO_STREAM_THROTTLE(0.5,"Plane Normal: " << cv::Point3f(planeNormalMap.x(),planeNormalMap.y(),planeNormalMap.z()) << " Point: " << cv::Point3f(planePointMap.x(),planePointMap.y(),planePointMap.z()));
    */
    tf::Vector3 planePointBase(testPlaneNormal_[0]*testPlaneDistance_,testPlaneNormal_[1]*testPlaneDistance_,testPlaneNormal_[2]*testPlaneDistance_);
    tf::Vector3 robotPosBase(0,0,0);
    tf::Vector3 planePointMap = base2map*planePointBase;
    tf::Vector3 robotPosMap = base2map*robotPosBase;

    tf::Vector3 diff = planePointMap-robotPosMap;

    cv::Point3f cvPlaneP(planePointMap.x(),planePointMap.y(),planePointMap.z());
    cv::Point3f cvPlaneN(diff.x(),diff.y(),diff.z());
    proc_.SetPlane(cvPlaneP,cvPlaneN);
    cv::Point3f tDir(diff.x(),diff.y(),diff.z());
    tDir = tDir* (1.0/sqrt(tDir.dot(tDir)));
    //ROS_INFO_STREAM_THROTTLE(0.5,"Plane Point: " << cv::Point3f(planePointMap.x(),planePointMap.y(),planePointMap.z()) << " Normal: " << tDir);
    //ROS_INFO_STREAM_THROTTLE(0.5,"planePointMap: " << cv::Point3f(planePointMap.x(),planePointMap.y(),planePointMap.z()) << " robotPosMap: " <<  cv::Point3f(robotPosMap.x(),robotPosMap.y(),robotPosMap.z()) << " Normal: " << tDir);


    cv::Point2f robotPos;
    robotPos.x = (base2map.getOrigin().x());
    robotPos.y = (base2map.getOrigin().y());

    cv::Point2d robotPosD;
    robotPosD.x = (base2map.getOrigin().x());
    robotPosD.y = (base2map.getOrigin().y());


    tf::Matrix3x3 rotMat(base2map.getRotation());
    double bRoll,bPitch,bYaw;
    rotMat.getRPY(bRoll,bPitch,bYaw);

    timeval tZstart;
    gettimeofday(&tZstart, NULL);


    tf::Transform cam2map;
    cam2map = base2map*cam2Bases_[idx];




    cv::Mat cvDepth = cv_bridge::toCvShare(depth,"")->image;


    SetupMatrices(cam2map);


    if (cvDepth.type() != CV_32F)
    {
        cv::Mat convImg;
        cvDepth.convertTo(convImg,CV_32F,1.0/1000.0,0);
        cvDepth = convImg;
    }

    if (removeLeftImageCols_ > 0)
    {
        UtilsDepthImage::RemoveLeftBorderNoise(cvDepth,removeLeftImageCols_,0);
    }

    if (removeWindowStartCol_ >= 0 && removeWindowEndCol_ >= 0 && removeWindowStartRow_ >= 0 && removeWindowEndRow_ >= 0)
    {
        UtilsDepthImage::RemoveWindow(cvDepth,removeWindowStartCol_,removeWindowEndCol_,removeWindowStartRow_,removeWindowEndRow_);
    }




    blockMap_.ReCenter(robotPos*(1.0));

    if (initBlockMap_)
    {
        blockMap_.SetSafeBlocksTo();
        initBlockMap_ = false;
    }


    proc_.minXVal_ = blockMap_.origin_.x;
    proc_.minYVal_ = blockMap_.origin_.y;



    cv::Vec4i minMax;


    switch (processMode_) {
    case PM_NN: proc_.ProcessDepthImageNN(cvDepth,cZImg_, cAssign_,minMax, mapScale_,mapOffset_,mapZeroLevel_);  break;
    case PM_MAX: proc_.ProcessDepthImageMaxNN(cvDepth,cZImg_, cAssign_,minMax, mapScale_,mapOffset_,mapZeroLevel_);  break;
    default:proc_.ProcessDepthImage(cvDepth,cZImg_, cAssign_,minMax, mapScale_,mapOffset_,mapZeroLevel_);  break;
    }

    cv::Mat resultImg;

    switch (fuseMode_) {
    case FM_TEMPORAL: {blockMap_.currentMap_.copyTo(resultImg); UpdateLocalMapTemporal(blockMap_.currentMap_,resultImg,cZImg_, cAssign_,minMax,cvPlaneP,cvPlaneN); /*resultImg = blockMap_.currentMap_;*/  break;}
    case FM_MAX: {UpdateLocalMapMax(blockMap_.currentMap_,cZImg_, cAssign_,minMax); resultImg = blockMap_.currentMap_;  break;}
    default:
    {
        if (processMode_ != PM_MAX){ UpdateLocalMapOverwrite(blockMap_.currentMap_,cZImg_, cAssign_,minMax); resultImg = blockMap_.currentMap_;}
        else {UpdateLocalMapOverwriteMax(blockMap_.currentMap_,cZImg_, cAssign_,minMax); resultImg = blockMap_.currentMap_;}
        break;
    }
    }


    std::string resultFrameID = localMapFrame_;

    if (transform2BaseLink_)
    {
        blockMap_.Transform2BaseLink(robotPosD,bYaw);
        resultImg = blockMap_.baseLinkMap_;
        resultFrameID = baseFrame_;

    }

    if (output16U_)
    {
        cv::Mat tempImg;
        resultImg.convertTo(tempImg,CV_16U);
        resultImg = tempImg;

    }



    if (postProcessType_ > 0)
    {
        cv::Mat tempImg;

        switch (postProcessType_) {
        case 1: cv::medianBlur(resultImg,tempImg,postProcessSize_);  break;
        case 2: {cv::Mat structElem = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(postProcessSize_,postProcessSize_)); cv::morphologyEx(resultImg,tempImg,cv::MORPH_OPEN,structElem); }  break;
        case 3: {cv::Mat structElem = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(postProcessSize_,postProcessSize_)); cv::morphologyEx(resultImg,tempImg,cv::MORPH_CLOSE,structElem); }  break;
        default: tempImg=resultImg; break;
        }
        resultImg = tempImg;


    }




    timeval tZend;
    gettimeofday(&tZend, NULL);


    float zImgMsElapsed = (float)(tZend.tv_sec - tZstart.tv_sec)*1000.0+ (float)(tZend.tv_usec - tZstart.tv_usec)/1000.0;
    numRegistered_++;
    totalRegisterTime_ += zImgMsElapsed;
    ROS_INFO_STREAM_THROTTLE(3,"ZImage Register Time: " << zImgMsElapsed << " Number: " << numRegistered_ << " AVG: " << totalRegisterTime_/(double)numRegistered_);


    geometry_msgs::PoseStamped localMapPose;

    localMapPose.pose.position.x = blockMap_.origin_.x;
    localMapPose.pose.position.y = blockMap_.origin_.y;
    localMapPose.pose.position.z = 0;

    localMapPose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

    localMapPose.header.stamp = timeStamp;
    localMapPose.header.frame_id = localMapFrame_;

    tf::Transform tfPose;
    tf::poseMsgToTF(localMapPose.pose,tfPose);
    static tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(tfPose, timeStamp, mapFrame_, localMapFrame_));



#ifdef ELEVATION_CLOUD_DEBUG
    /*
    // for testing plane reject
    std::vector<cv::Point3f> points;
    for (int yl  = -100; yl < 100;yl++)
    {
        for (int xl  = -100; xl < 100;xl++)
        {
            cv::Point3f tp;

            tp.x = robotPos.x + ((float)xl)*0.02;
            tp.y = robotPos.y + ((float)yl)*0.02;
            tp.z = 0.5f;

            if (!proc_.TestPlane(tp.x, tp.y, tp.z)) points.push_back(tp);
        }
    }

    if (imageCloud_pub_.getNumSubscribers() > 0) UtilsDem2PC::CreateCloud(points,mapFrame_,timeStamp,imageCloud_pub_);
    */
    if (imageCloud_pub_.getNumSubscribers() > 0) UtilsDem2PC::PublishCloud(timeStamp,mapFrame_,blockMap_.currentMap_,imageCloud_pub_,blockMap_.origin_, blockMap_.pixelResolution_);
#endif

    if (zImagePub_.getNumSubscribers() > 0)
    {
        cv_bridge::CvImage out_z_image;
        out_z_image.header   = depth->header; // Same timestamp and tf frame as input image
        out_z_image.header.stamp   = timeStamp; // Same timestamp and tf frame as input image
        out_z_image.header.frame_id   = resultFrameID; // Same timestamp and tf frame as input image
        if (resultImg.type() == CV_32F)
        {
            out_z_image.encoding = sensor_msgs::image_encodings::TYPE_32FC1; // Or whatever
        }
        else
        {
            out_z_image.encoding = sensor_msgs::image_encodings::TYPE_16UC1; // Or whatever
        }
        out_z_image.image    = resultImg; // Your cv::Mat
        zImagePub_.publish(out_z_image.toImageMsg());
    }


    if (assignImagePub_.getNumSubscribers() > 0)
    {

        cv_bridge::CvImage out_assign_image;
        out_assign_image.header   = depth->header; // Same timestamp and tf frame as input image
        out_assign_image.header.stamp   = timeStamp; // Same timestamp and tf frame as input image
        out_assign_image.encoding = sensor_msgs::image_encodings::TYPE_32FC1; // Or whatever
        out_assign_image.image    = cAssign_; // Your cv::Mat
        assignImagePub_.publish(out_assign_image.toImageMsg());
    }


}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "LocalMap_Node");
    LocalmapMC demNode;

    ros::Rate r(60);
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

