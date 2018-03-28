#include "plannerbase.h"
#include "utils_draw.h"

PlannerBase::PlannerBase()
{
    bestNode_ = nullptr;

    path_.clear();


}

Trajectory* PlannerBase::GetResultTrajectory()
{

    if (bestNode_ == nullptr) return nullptr;

    TrajNode* curPtr = bestNode_;

    std::vector<TrajNode*> ptrList;
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
            if (curPtr->poseResults_[i].validState !=  PERS_NOTASSIGNED)resultTraj_.poseResults_.push_back(curPtr->poseResults_[i]);
        }


        //resultTraj_.poseResults_.insert(resultTraj_.poseResults_.end(),curPtr->poseResults_.begin(),curPtr->poseResults_.end());
    }

    return &resultTraj_;
}

Trajectory* PlannerBase::GetBLResultTrajectory()
{
    Trajectory* imgBestTraj = GetResultTrajectory();

    if (imgBestTraj == nullptr) return nullptr;

    resultBLTraj_.Reset(0);
    resultBLTraj_.poseResults_.reserve(imgBestTraj->poseResults_.size());


    for (unsigned int tl = 0;tl < resultTraj_.poseResults_.size();++tl)
    {
        PoseEvalResults curRes = resultTraj_.poseResults_[tl];
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




cv::Mat PlannerBase::DrawDebugImage(float scalingFactor, bool drawRobot)
{
    DrawProc dp;
    dp.drawZMin_ = - config_.procConfig_.mapBaseHeight/30;
    dp.drawZMax_ =  config_.procConfig_.mapBaseHeight/30;


    cv::Mat dem = poseEstimator_.GetDEM();

    cv::Mat drawMat = dp.D16SImageToRGB(dem,config_.procConfig_.mapBaseHeight+ dp.drawZMin_,config_.procConfig_.mapBaseHeight+dp.drawZMax_);

    dp.DrawMapStates(dem,drawMat,config_.procConfig_);


    ScaledDrawProc sdp;

    dp.SetupDrawProc(sdp,drawMat,scalingFactor);

    PoseEvalResults result;

    result.Reset();

    IModelBasedPlanner* iplanner = (IModelBasedPlanner*)this;

    std::vector<TrajNode*> trajectories_;
    iplanner->GetAllTrajectoryNodes(trajectories_);

    if (trajectories_.size() > 0)
    {
        TrajNode* curTraj = trajectories_[0];
        result = *curTraj->start_;
    }


    result.SetWheelAnglesGlobal(result.pose.z);
    result.wheelEvalResults_[0].wheelAngleIdx = poseEstimator_.GetRobotModel()->GetAngleIdx(result.pose.z);
    result.wheelEvalResults_[1].wheelAngleIdx = poseEstimator_.GetRobotModel()->GetAngleIdx(result.pose.z);
    result.wheelEvalResults_[2].wheelAngleIdx = poseEstimator_.GetRobotModel()->GetAngleIdx(result.pose.z);
    result.wheelEvalResults_[3].wheelAngleIdx = poseEstimator_.GetRobotModel()->GetAngleIdx(result.pose.z);



    if (drawRobot) dp.DrawRobotScaled(sdp,*poseEstimator_.GetRobotModel(),result);


    dp.DrawTrajectories(sdp,trajectories_,iplanner->GetResultTrajectory());


    dp.DrawGoal(sdp,goal_,curImgRobotPose_);
    dp.DrawPath(sdp,path_);


    return sdp.GetImage();


}


