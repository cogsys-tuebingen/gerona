#include "poseestimator.h"
#include "utils_draw.h"

PoseEstimator::PoseEstimator()
{
    demPtr_ = nullptr;

    //poseCounter_ = 0;
    //saveXPos_ = 100;
}


void PoseEstimator::Setup(ModelBasedPlannerConfig &config)
{
    //procConfig_ = config.procConfig_;
    //scorerConfig_ = config.scorerConfig_;
    robotModel_.SetupRobot(config);
}



void PoseEstimator::SetDem(CVAlignedMat::ptr dem)
{
    demPtr_ = dem;

    dem_ = demPtr_->mat_;

}
void PoseEstimator::SetDem(cv::Mat dem)
{
    demPtr_ = CVAlignedMat::Create(dem);

    dem_ = demPtr_->mat_;

}


void PoseEstimator::Evaluate(PoseEvalResults &results) const
{
    //const int res = robotModel_.EvaluatePose(dem_,results);
    robotModel_.EvaluatePose(dem_,results);

    //++poseCounter_;

    //CheckState(results);
    //results.validState = 0;

    /*
    if (fabs(results.zValues[0]) > procConfig_.validThreshold ) results.validState = PERS_NOWHEELSUPPORT;
    if (fabs(results.zValues[1]) > procConfig_.validThreshold ) results.validState = PERS_NOWHEELSUPPORT;
    if (fabs(results.zValues[2]) > procConfig_.validThreshold ) results.validState = PERS_NOWHEELSUPPORT;
    if (fabs(results.zValues[3]) > procConfig_.validThreshold ) results.validState = PERS_NOWHEELSUPPORT;

    if (saveXPos_ > 0 && results.validState == PERS_OUTOFIMAGE && results.pose.x < saveXPos_)
    {
        results.validState = PERS_SAFE_OUTOFIMAGE;
        results.gravAngle = 1.0;
        results.tipAngle = 1.0;
        results.n1.x = 0;
        results.n1.y = 0;
        results.n1.z = 1;
        results.n2.x = 0;
        results.n2.y = 0;
        results.n2.z = 1;


    }
    */
    //if(results.validState < 0) results.Reset();

}

cv::Mat PoseEstimator::DrawDebugImage(PoseEvalResults &results)
{
    DrawProc dp;
    ProcConfig procConfig_ = robotModel_.GetProcConfig();

    dp.drawZMin_ = - procConfig_.mapBaseHeight/30;
    dp.drawZMax_ =  procConfig_.mapBaseHeight/30;

    cv::Mat dem = dem_;


    cv::Mat drawMat = dp.D16SImageToRGB(dem,procConfig_.mapBaseHeight+ dp.drawZMin_,procConfig_.mapBaseHeight+dp.drawZMax_);

    //dp.DrawMapStates(dem,drawMat,procConfig_);


    ScaledDrawProc sdp;

    dp.SetupDrawProc(sdp,drawMat,1.0f);

    dp.DrawRobotScaled(sdp,robotModel_,results);

    return sdp.GetImage();
}


/*
int PoseEstimator::CheckState(PoseEvalResults &results)
{


    if (results.validState == PERS_OUTOFIMAGE)
    {
        if (saveXPos_ > 0 && results.pose.x < saveXPos_)
        {
            results.validState = PERS_SAFE_OUTOFIMAGE;
            results.gravAngle = 1.0;
            results.tipAngle = 1.0;
            results.n1.x = 0;
            results.n1.y = 0;
            results.n1.z = 1;
            results.n2.x = 0;
            results.n2.y = 0;
            results.n2.z = 1;

            return 0;
        }


        return 0;
    }


    if (results.TestWheelZValues(procConfig_.validThreshold))
    {
        results.validState = PERS_NOWHEELSUPPORT;
        results.gravAngle = 0.0;
        results.tipAngle = 0.0;
        results.n1.x = 1;
        results.n1.y = 0;
        results.n1.z = 0;
        results.n2.x = 0;
        results.n2.y = 1;
        results.n2.z = 0;

        return 0;
    }

    if (results.TestWheelZValues(procConfig_.notVisibleThreshold))
    {
        results.validState = PERS_NOTVISIBLE;
        results.gravAngle = 1.0;
        results.tipAngle = 1.0;
        results.n1.x = 0;
        results.n1.y = 0;
        results.n1.z = 1;
        results.n2.x = 0;
        results.n2.y = 0;
        results.n2.z = 1;
        results.wheelEvalResults_[0].wheelSupport = 1.0f;
        results.wheelEvalResults_[1].wheelSupport = 1.0f;
        results.wheelEvalResults_[2].wheelSupport = 1.0f;
        results.wheelEvalResults_[3].wheelSupport = 1.0f;



        return 0;
    }


    if (results.gravAngle < scorerConfig_.gravAngleThreshold)
    {
        results.validState = PERS_EXCEEDANGLE;
    }
    if (results.deltaAngle > scorerConfig_.deltaAngleThreshold)
    {
        results.validState = PERS_EXCEEDANGLE;
    }


    return 0;

}
*/


/*
void PoseEstimator::EvaluateNP(PoseEvalResults &results)
{
    //robotModel_.EvaluatePoseNP(dem_,results);

    //CheckState(results);

}

void PoseEstimator::EvaluateWS(PoseEvalResults &results)
{
    //robotModel_.EvaluatePoseWS(dem_,results);

    //CheckState(results);

}
*/
