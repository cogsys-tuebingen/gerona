#include "poseestimator_dt.h"
#include "utils_draw_dt.h"

PoseEstimatorDT::PoseEstimatorDT()
{
    dmapPtr_ = nullptr;

    //poseCounter_ = 0;
    //saveXPos_ = 100;
}


void PoseEstimatorDT::Setup(DTPlannerConfig &config)
{
    //procConfig_ = config.procConfig_;
    //scorerConfig_ = config.scorerConfig_;
    robotModel_.SetupRobot(config);
}


/*
void PoseEstimator::SetDem(CVAlignedMat::ptr dem)
{
    demPtr_ = dem;

    dem_ = demPtr_->mat_;

}
*/


void PoseEstimatorDT::SetDem(cv::Mat dem)
{
    orgDem_ = dem;

    /*
    cv::Mat maskImg(dem.rows,dem.cols,CV_8U);
    //dem.copyTo(tDem);

    unsigned char* dstPtr;
    const short *srcPtr;

    for (unsigned int y = 0; y < dem.rows;++y)
    {
        srcPtr = dem.ptr<short>(y);
        dstPtr = maskImg.ptr<unsigned char>(y);
        for (unsigned int x = 0; x < dem.cols;++x)
        {
            if (srcPtr[x] < 9900 || srcPtr[x] > 10100)
            {
                dstPtr[x] = 0;
            }
            else dstPtr[x] = 1;
        }
    }

    cv::Mat res;
    cv::distanceTransform(maskImg,res,CV_DIST_L2,CV_DIST_MASK_PRECISE);




    dmapPtr_ = CVAlignedMat::Create(res);

    dmap_ = dmapPtr_->mat_;
    */

}

void PoseEstimatorDT::CreateMap(const cv::Point3f &rPose, const PlannerScorerConfigDT& scoreConf )
{
    ProcConfigDT procConfig_ = robotModel_.GetProcConfig();
    //RobotConfigDT robotConfig_ = robotModel_.GetRobotConfig();

    cv::Point2f rPos(rPose.x,rPose.y);

    //short lowerLimit = (short)(procConfig_.mapBaseHeight - scoreConf.maxDownStep*procConfig_.heightScale);
    //short upperLimit = (short)(procConfig_.mapBaseHeight + scoreConf.maxUpStep*procConfig_.heightScale);

    float wheelSupportFarThreshold = scoreConf.noWheelSupportNearThreshold * procConfig_.pixelSizeInv;

    if (wheelSupportFarThreshold < 0) wheelSupportFarThreshold = 100000;

    cv::Mat maskImg(orgDem_.rows,orgDem_.cols,CV_8U);
    //dem.copyTo(tDem);

    unsigned char* dstPtr;
    const short *srcPtr;

    cv::Point2f pixelPos;
    cv::Point2f diffPos;

    const short notVisibleLevel = (short)procConfig_.notVisibleLevel;
    const float wheelSupportFarThresholdSqr = wheelSupportFarThreshold*wheelSupportFarThreshold;
    const float pixelSizeSqr = procConfig_.pixelSize* procConfig_.pixelSize;
    const float distPowSqr = 1.0f/(scoreConf.heihgtLimitPowPerMeter*scoreConf.heihgtLimitPowPerMeter);
    float srcVal;


    for (unsigned int y = 0; y < orgDem_.rows;++y)
    {
        srcPtr = orgDem_.ptr<short>(y);
        dstPtr = maskImg.ptr<unsigned char>(y);
        for (unsigned int x = 0; x < orgDem_.cols;++x)
        {
            pixelPos.x = (float)x;
            pixelPos.y = (float)y;
            diffPos = rPos-pixelPos;


            const float distanceSqr = (diffPos.dot(diffPos));
            const float distM = distanceSqr*pixelSizeSqr;
            float limitScale = (1.0f + (distM-wheelSupportFarThresholdSqr)*distPowSqr);
            if (limitScale < 1.0f) limitScale = 1.0f;

            const float lowerLimitF = procConfig_.mapBaseHeight - scoreConf.maxDownStep*procConfig_.heightScale*limitScale;
            const float upperLimitF = procConfig_.mapBaseHeight + scoreConf.maxUpStep*  procConfig_.heightScale*limitScale;

            srcVal = (float)srcPtr[x];
            dstPtr[x] = 1;

            if (srcVal < lowerLimitF)
            {
                if (!(srcPtr[x] <= notVisibleLevel && (distanceSqr > wheelSupportFarThresholdSqr) ))
                    dstPtr[x] = 0;
            }
            if (srcVal > upperLimitF)
            {
                dstPtr[x] = 0;
            }
        }
    }

    cv::Mat res;
    cv::distanceTransform(maskImg,res,CV_DIST_L2,CV_DIST_MASK_PRECISE);




    dmapPtr_ = CVAlignedMat::Create(res);

    dmap_ = dmapPtr_->mat_;


}

/*
void PoseEstimatorDT::CreateMap(const cv::Point3f &rPose, const PlannerScorerConfigDT& scoreConf )
{
    ProcConfigDT procConfig_ = robotModel_.GetProcConfig();
    //RobotConfigDT robotConfig_ = robotModel_.GetRobotConfig();

    cv::Point2f rPos(rPose.x,rPose.y);

    //short lowerLimit = (short)(procConfig_.mapBaseHeight - scoreConf.maxDownStep*procConfig_.heightScale);
    //short upperLimit = (short)(procConfig_.mapBaseHeight + scoreConf.maxUpStep*procConfig_.heightScale);

    float wheelSupportFarThreshold = scoreConf.noWheelSupportNearThreshold * procConfig_.pixelSizeInv;

    if (wheelSupportFarThreshold < 0) wheelSupportFarThreshold = 100000;

    cv::Mat maskImg(orgDem_.rows,orgDem_.cols,CV_8U);
    //dem.copyTo(tDem);

    unsigned char* dstPtr;
    const short *srcPtr;

    cv::Point2f pixelPos;
    cv::Point2f diffPos;

    const short notVisibleLevel = (short)procConfig_.notVisibleLevel;
    const float wheelSupportFarThresholdSqr = wheelSupportFarThreshold*wheelSupportFarThreshold;

    float srcVal;


    for (unsigned int y = 0; y < orgDem_.rows;++y)
    {
        srcPtr = orgDem_.ptr<short>(y);
        dstPtr = maskImg.ptr<unsigned char>(y);
        for (unsigned int x = 0; x < orgDem_.cols;++x)
        {
            pixelPos.x = x;
            pixelPos.y = y;
            diffPos = rPos-pixelPos;
            srcVal = (float)srcPtr[x];

            dstPtr[x] = 1;

            const float distance = std::sqrt(diffPos.dot(diffPos));
            const float distM = distance*procConfig_.pixelSize;
            float limitScale = std::pow(scoreConf.heihgtLimitPowPerMeter,((distM-scoreConf.noWheelSupportNearThreshold)/scoreConf.noWheelSupportNearThreshold));
            if (limitScale < 1.0f) limitScale = 1.0f;

            const float lowerLimitF = procConfig_.mapBaseHeight - scoreConf.maxDownStep*procConfig_.heightScale*limitScale;
            const float upperLimitF = procConfig_.mapBaseHeight + scoreConf.maxUpStep*  procConfig_.heightScale*limitScale;



            if (srcVal < lowerLimitF)
            {
                if (!(srcPtr[x] <= notVisibleLevel && (distance > wheelSupportFarThreshold) ))
                    dstPtr[x] = 0;
            }
            if (srcVal > upperLimitF)
            {
                dstPtr[x] = 0;
            }
        }
    }

    cv::Mat res;
    cv::distanceTransform(maskImg,res,CV_DIST_L2,CV_DIST_MASK_PRECISE);




    dmapPtr_ = CVAlignedMat::Create(res);

    dmap_ = dmapPtr_->mat_;


}
*/

void PoseEstimatorDT::Evaluate(PoseEvalResultsDT &results) const
{

    robotModel_.EvaluatePose(dmap_,results);


}

cv::Mat PoseEstimatorDT::DrawDebugImage(PoseEvalResultsDT &results)
{
    DrawProc dp;
    ProcConfigDT procConfig_ = robotModel_.GetProcConfig();

    dp.drawZMin_ = - procConfig_.mapBaseHeight/20;
    dp.drawZMax_ =  procConfig_.mapBaseHeight/20;

    cv::Mat dem = dmap_;


    cv::Mat drawMat = dp.D16SImageToRGB(dem,procConfig_.mapBaseHeight+ dp.drawZMin_,procConfig_.mapBaseHeight+dp.drawZMax_);

    //dp.DrawMapStates(dem,drawMat,procConfig_);


    ScaledDrawProc sdp;

    dp.SetupDrawProc(sdp,drawMat,1.0f);

    dp.DrawRobotScaled(sdp,robotModel_,results);

    return sdp.GetImage();
}
