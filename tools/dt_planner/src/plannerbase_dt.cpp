#include "plannerbase_dt.h"
#include "utils_draw_dt.h"

PlannerBaseDT::PlannerBaseDT()
{
    bestNode_ = nullptr;

    path_.clear();


}

TrajectoryDT* PlannerBaseDT::GetResultTrajectory()
{

    if (bestNode_ == nullptr) return nullptr;

    TrajNodeDT* curPtr = bestNode_;

    std::vector<TrajNodeDT*> ptrList;
    int numPoses = 0;
    while (curPtr->parent_ != nullptr)
    {
        ptrList.push_back(curPtr);
        numPoses+= curPtr->numValid_;
        curPtr = curPtr->parent_;
    }



    resultTraj_.Reset(0);
    resultTraj_.poseResults_.reserve(numPoses);


    for (int tl = (ptrList.size()-1);tl >= 0;--tl)
    {
        curPtr = ptrList[tl];

        for (unsigned int i = 0; i < curPtr->poseResults_.size();++i)
        {
            if (curPtr->poseResults_[i].validState !=  PERSDT_NOTASSIGNED)resultTraj_.poseResults_.push_back(curPtr->poseResults_[i]);
        }


        //resultTraj_.poseResults_.insert(resultTraj_.poseResults_.end(),curPtr->poseResults_.begin(),curPtr->poseResults_.end());
    }

    return &resultTraj_;
}

TrajectoryDT* PlannerBaseDT::GetBLResultTrajectory()
{
    TrajectoryDT* imgBestTraj = GetResultTrajectory();

    if (imgBestTraj == nullptr) return nullptr;

    resultBLTraj_.Reset(0);
    resultBLTraj_.poseResults_.reserve(imgBestTraj->poseResults_.size());


    for (unsigned int tl = 0;tl < resultTraj_.poseResults_.size();++tl)
    {
        PoseEvalResultsDT curRes = resultTraj_.poseResults_[tl];
        curRes.pose = ImgPoseToPose(curRes.pose);
        curRes.cmd.x *= config_.procConfig_.pixelSize;
        resultBLTraj_.poseResults_.push_back(curRes);

    }

    if (resultBLTraj_.poseResults_.size() > 0)
    {
        resultBLTraj_.start_ = &resultBLTraj_.poseResults_[0];
        resultBLTraj_.end_ = &resultBLTraj_.poseResults_[resultBLTraj_.poseResults_.size()-1];
    }

    return &resultBLTraj_;
}




cv::Mat PlannerBaseDT::DrawDebugImage(float scalingFactor, bool drawRobot)
{
    DrawProc dp;
    dp.conf = config_;
    //dp.drawZMin_ = - config_.procConfig_.mapBaseHeight/20;
    //dp.drawZMax_ =  config_.procConfig_.mapBaseHeight/20;


    cv::Mat dem = poseEstimator_.GetDEM();

    short minVal = 32000,maxVal = 0;
    short *demPtr;
    for (int y = 0; y < dem.rows;++y)
    {
        demPtr = dem.ptr<short>(y);
        for (int x = 0; x < dem.cols;++x)
        {
            if (demPtr[x] > config_.procConfig_.notVisibleLevel)
            {
                if (demPtr[x] > maxVal) maxVal = demPtr[x];
                if (demPtr[x] < minVal) minVal = demPtr[x];

            }
        }
    }
    dp.drawZMin_ = minVal- config_.procConfig_.mapBaseHeight;
    dp.drawZMax_ = maxVal- config_.procConfig_.mapBaseHeight;


    cv::Mat drawMat = dp.D16SImageToRGB(dem,config_.procConfig_.mapBaseHeight+ dp.drawZMin_,config_.procConfig_.mapBaseHeight+dp.drawZMax_);

    dp.DrawMapStates(dem,drawMat,config_.procConfig_);

    cv::Mat dmap = poseEstimator_.GetDMap();


    float *dmapPtr;
    cv::Vec3b *dstPtr;
    for (int y = 0; y < dmap.rows;++y)
    {
        dmapPtr = dmap.ptr<float>(y);
        dstPtr = drawMat.ptr<cv::Vec3b>(y);
        for (int x = 0; x < dmap.cols;++x)
        {
            if (dmapPtr[x] < config_.scorerConfig_.distanceThreshold)
            {
                dstPtr[x][2] = 255;

            }
            else
            {
                dstPtr[x][0] = dstPtr[x][1]+dmapPtr[x] < 255?dstPtr[x][1]+dmapPtr[x]:255 ;

            }
        }
    }



    ScaledDrawProc sdp;

    dp.SetupDrawProc(sdp,drawMat,scalingFactor);

    PoseEvalResultsDT result;

    result.Reset();

    IDTPlanner* iplanner = (IDTPlanner*)this;

    std::vector<TrajNodeDT*> trajectories_;
    iplanner->GetAllTrajectoryNodes(trajectories_);

    if (bestNode_ != nullptr)
    {
        TrajNodeDT* curTraj = bestNode_->GetFirstNode();
        result = *curTraj->start_;
    }
    else if (trajectories_.size() > 0)
    {
        TrajNodeDT* curTraj = trajectories_[0];
        result = *curTraj->start_;
    }


    /*
    result.SetWheelAnglesGlobal(result.pose.z);
    result.wheelEvalResults_[0].wheelAngleIdx = poseEstimator_.GetRobotModel()->GetAngleIdx(result.pose.z);
    result.wheelEvalResults_[1].wheelAngleIdx = poseEstimator_.GetRobotModel()->GetAngleIdx(result.pose.z);
    result.wheelEvalResults_[2].wheelAngleIdx = poseEstimator_.GetRobotModel()->GetAngleIdx(result.pose.z);
    result.wheelEvalResults_[3].wheelAngleIdx = poseEstimator_.GetRobotModel()->GetAngleIdx(result.pose.z);
*/


    if (drawRobot) dp.DrawRobotScaled(sdp,*poseEstimator_.GetRobotModel(),result);


    dp.DrawTrajectories(sdp,trajectories_,iplanner->GetResultTrajectory());


    dp.DrawGoal(sdp,goal_,curImgRobotPose_);
    dp.DrawPath(sdp,path_);


    return sdp.GetImage();


}

cv::Mat PlannerBaseDT::DrawDebugImage(PoseEvalResultsDT result, float scalingFactor, bool drawRobot)
{
    DrawProc dp;
    dp.conf = config_;

    //dp.drawZMin_ = - config_.procConfig_.mapBaseHeight/20;
    //dp.drawZMax_ =  config_.procConfig_.mapBaseHeight/20;



    cv::Mat dem = poseEstimator_.GetDEM();

    short minVal = 32000,maxVal = 0;
    short *demPtr;
    for (int y = 0; y < dem.rows;++y)
    {
        demPtr = dem.ptr<short>(y);
        for (int x = 0; x < dem.cols;++x)
        {
            if (demPtr[x] > config_.procConfig_.notVisibleLevel)
            {
                if (demPtr[x] > maxVal) maxVal = demPtr[x];
                if (demPtr[x] < minVal) minVal = demPtr[x];

            }
        }
    }
    dp.drawZMin_ = minVal- config_.procConfig_.mapBaseHeight;
    dp.drawZMax_ = maxVal- config_.procConfig_.mapBaseHeight;


    cv::Mat drawMat = dp.D16SImageToRGB(dem,config_.procConfig_.mapBaseHeight+ dp.drawZMin_,config_.procConfig_.mapBaseHeight+dp.drawZMax_);

    dp.DrawMapStates(dem,drawMat,config_.procConfig_);

    cv::Mat dmap = poseEstimator_.GetDMap();


    float *dmapPtr;
    cv::Vec3b *dstPtr;
    for (int y = 0; y < dmap.rows;++y)
    {
        dmapPtr = dmap.ptr<float>(y);
        dstPtr = drawMat.ptr<cv::Vec3b>(y);
        for (int x = 0; x < dmap.cols;++x)
        {
            if (dmapPtr[x] < config_.scorerConfig_.distanceThreshold)
            {
                dstPtr[x][2] = 255;

            }
            else
            {
                dstPtr[x][1] = dstPtr[x][1]+dmapPtr[x] < 255?dstPtr[x][1]+dmapPtr[x]:255 ;

            }
        }
    }


    ScaledDrawProc sdp;

    dp.SetupDrawProc(sdp,drawMat,scalingFactor);

    /*
    result.SetWheelAnglesGlobal(result.pose.z);
    result.wheelEvalResults_[0].wheelAngleIdx = poseEstimator_.GetRobotModel()->GetAngleIdx(result.pose.z);
    result.wheelEvalResults_[1].wheelAngleIdx = poseEstimator_.GetRobotModel()->GetAngleIdx(result.pose.z);
    result.wheelEvalResults_[2].wheelAngleIdx = poseEstimator_.GetRobotModel()->GetAngleIdx(result.pose.z);
    result.wheelEvalResults_[3].wheelAngleIdx = poseEstimator_.GetRobotModel()->GetAngleIdx(result.pose.z);
*/


    if (drawRobot) dp.DrawRobotScaled(sdp,*poseEstimator_.GetRobotModel(),result);


    //dp.DrawTrajectories(sdp,trajectories_,iplanner->GetResultTrajectory());


    //dp.DrawGoal(sdp,goal_,curImgRobotPose_);
    //dp.DrawPath(sdp,path_);


    return sdp.GetImage();


}


