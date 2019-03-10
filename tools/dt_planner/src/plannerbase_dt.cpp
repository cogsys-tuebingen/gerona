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


cv::Mat PlannerBaseDT::DrawMap(PoseEvalResultsDT &results)
{
    DrawProc dp;
    cv::Mat dmap = poseEstimator_.GetDMap();
    cv::Mat dem = poseEstimator_.GetDEM();

    cv::Mat drawMat = dp.D16SImageToRGB(dem,config_.procConfig_.mapBaseHeight- config_.procConfig_.heightScale,config_.procConfig_.mapBaseHeight+ config_.procConfig_.heightScale);

    //dp.DrawMapStates(dem,drawMat,config_.procConfig_);



    const float *dmapPtr;
    cv::Vec3b *dstPtr;
    const short *demPtr;

    cv::Point2f rPos(results.pose.x,results.pose.y);
    cv::Point2f pPos;
    cv::Point2f diffPos;
    float distP = 0;

    const short notvisibleLevelS = config_.procConfig_.notVisibleLevel;

    const float noWSSqr = config_.scorerConfig_.noWheelSupportNearThresholdImg*config_.scorerConfig_.noWheelSupportNearThresholdImg;

    const float dontCareDistanceImgInv = 1.0f / config_.scorerConfig_.dontCareDistanceImg;

    for (int y = 0; y < dmap.rows;++y)
    {
        dmapPtr = dmap.ptr<float>(y);
        dstPtr = drawMat.ptr<cv::Vec3b>(y);
        demPtr = dem.ptr<short>(y);
        for (int x = 0; x < dmap.cols;++x)
        {
            pPos.x = x;
            pPos.y = y;
            diffPos = pPos-rPos;
            distP = (diffPos.dot(diffPos));

            if (results.validState != PERSDT_NOTASSIGNED && demPtr[x] <= notvisibleLevelS && distP > noWSSqr)
            {

                    dstPtr[x][0] = 100;
                    dstPtr[x][1] = 100;

            }
            else
            {

                const float distNorm = 1.0f -(dmapPtr[x] > config_.scorerConfig_.dontCareDistanceImg? 1.0f : dmapPtr[x]*dontCareDistanceImgInv);

                //distNorm = 1.0f - distNorm;

                const float tCVal = dstPtr[x][0];

                const float resC = tCVal*(1.0f - (distNorm*0.5 ) );

                dstPtr[x][1] = resC;
                dstPtr[x][2] = resC;


            }

        }
    }

    return drawMat;
}

cv::Mat PlannerBaseDT::DrawDebugImageFast(float scalingFactor, bool drawRobot)
{
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

    cv::Mat dem = poseEstimator_.GetDEM();



    ScaledDrawProc sdp;
    DrawProc dp;
    dp.conf = config_;

    cv::Mat drawMat = dp.D16SImageToRGB(dem,config_.procConfig_.mapBaseHeight- config_.procConfig_.heightScale,config_.procConfig_.mapBaseHeight+ config_.procConfig_.heightScale);


    dp.SetupDrawProc(sdp,drawMat,scalingFactor);



    if (drawRobot) dp.DrawRobotScaled(sdp,*poseEstimator_.GetRobotModel(),result);


    dp.DrawTrajectoriesFast(sdp,trajectories_,iplanner->GetResultTrajectory());


    dp.DrawGoal(sdp,goal_,curImgRobotPose_);
    dp.DrawPath(sdp,path_);


    return sdp.GetImage();


}


cv::Mat PlannerBaseDT::DrawDebugImage(float scalingFactor, bool drawRobot)
{
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

    cv::Mat drawMat = DrawMap(result);


    ScaledDrawProc sdp;
    DrawProc dp;
    dp.conf = config_;


    dp.SetupDrawProc(sdp,drawMat,scalingFactor);



    if (drawRobot) dp.DrawRobotScaled(sdp,*poseEstimator_.GetRobotModel(),result);


    dp.DrawTrajectories(sdp,trajectories_,iplanner->GetResultTrajectory());


    dp.DrawGoal(sdp,goal_,curImgRobotPose_);
    dp.DrawPath(sdp,path_);


    return sdp.GetImage();


}

cv::Mat PlannerBaseDT::DrawDebugImage(PoseEvalResultsDT result, float scalingFactor, bool drawRobot)
{
    cv::Mat drawMat = DrawMap(result);


    ScaledDrawProc sdp;
    DrawProc dp;
    dp.conf = config_;


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


