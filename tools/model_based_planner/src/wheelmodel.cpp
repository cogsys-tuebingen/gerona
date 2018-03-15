#include "wheelmodel.h"
#include "wheelrender.h"
#include "utils_diff.h"

WheelModel::WheelModel()
{

}


int WheelModel::Evaluate(const cv::Mat &dem,const cv::Point2f &pos, WheelEvalResults &results) const
{
    const WheelDescriptor &desc = descriptors_[results.wheelAngleIdx];

    const cv::Point2i imagePos = GetImagePos(pos,desc);

    if (imagePos.x < 0 || imagePos.y < 0 ) return -1;

    const cv::Mat &wImg = desc.image_->mat_;

    if (imagePos.x + wImg.cols > dem.cols || imagePos.y + wImg.rows > dem.rows ) return -1;

    //results.zValue = Utils_DIFF::diffMinPos(dem,wImg,imagePos.x,imagePos.y,results.contactPoint.x,results.contactPoint.y);
    results.zValue = Utils_DIFF::np_diffMinPos(dem,wImg,imagePos.x,imagePos.y,results.contactPoint.x,results.contactPoint.y);


    const int wsPixels = Utils_DIFF::calcWheelSupport(dem,wImg,imagePos.x,imagePos.y,wheelSupportThreshold_,results.zValue);
    results.wheelSupport = (float)wsPixels*desc.numImagePixelsInv_;
    return 0;
}

/*
int WheelModel::Evaluate(const cv::Mat &dem,const cv::Point2f &pos, const int &angleIdx, int &zval, cv::Point2i &contactPoint) const
{
    const WheelDescriptor &desc = descriptors_[angleIdx];
    const cv::Point2f imageOrigin = pos- desc.jointPosImg_;
    cv::Point2i imgPos(round(imageOrigin.x),round(imageOrigin.y));

    if (imgPos.x < 0 || imgPos.y < 0 ) return -1;

    const cv::Mat &wImg = desc.image_->mat_;

    if (imgPos.x + wImg.cols > dem.cols || imgPos.y + wImg.rows > dem.rows ) return -1;

    zval = Utils_DIFF::diffMinPos(dem,wImg,imgPos.x,imgPos.y,contactPoint.x,contactPoint.y);
    return 0;
}

int WheelModel::EvaluateNP(const cv::Mat &dem,const cv::Point2f &pos, const int &angleIdx, int &zval, cv::Point2i &contactPoint) const
{
    contactPoint.x = 0;
    contactPoint.y = 0;
    const WheelDescriptor &desc = descriptors_[angleIdx];
    const cv::Point2f imageOrigin = pos- desc.jointPosImg_;
    cv::Point2i imgPos(round(imageOrigin.x),round(imageOrigin.y));

    if (imgPos.x < 0 || imgPos.y < 0 ) return -1;

    const cv::Mat wImg = desc.image_->mat_;

    if (imgPos.x + wImg.cols > dem.cols || imgPos.y + wImg.rows > dem.rows ) return -1;

    zval = Utils_DIFF::np_diffMinPos(dem,wImg,imgPos.x,imgPos.y,contactPoint.x,contactPoint.y);
    return 0;
}

int WheelModel::EvaluateWS(const cv::Mat &dem,const cv::Point2f &pos, const int &angleIdx, int &zval, cv::Point2i &contactPoint, float &wsRes) const
{
    const WheelDescriptor &desc = descriptors_[angleIdx];
    const cv::Point2f imageOrigin = pos- desc.jointPosImg_;
    cv::Point2i imgPos(round(imageOrigin.x),round(imageOrigin.y));

    if (imgPos.x < 0 || imgPos.y < 0 ) return -1;

    const cv::Mat &wImg = desc.image_->mat_;

    if (imgPos.x + wImg.cols > dem.cols || imgPos.y + wImg.rows > dem.rows ) return -1;

    int wsPixels;
    //const int wsThresh = wheelSupportThreshold_;
    zval = Utils_DIFF::ws_diffMinPos(dem,wImg,imgPos.x,imgPos.y,wheelSupportThreshold_,contactPoint.x,contactPoint.y,wsPixels);

    wsRes = (float)wsPixels/(float)desc.numImagePixels_;

    return 0;
}
*/


/*
short WheelModel::EvaluateShort(const cv::Mat &dem,const cv::Point2f &pos, const int &angleIdx , short &cx,short &cy)
{
    const WheelDescriptor desc = descriptors_[angleIdx];
    const cv::Point2f imageOrigin = pos- desc.jointPosImg_;
    cv::Point2i imgPos(round(imageOrigin.x),round(imageOrigin.y));

    if (imgPos.x < 0 || imgPos.y < 0 ) return -1;

    const cv::Mat wImg = desc.image_->mat_;

    if (imgPos.x + wImg.cols > dem.cols || imgPos.y + wImg.rows > dem.rows ) return -1;


    return Utils_DIFF::diffMinPosShort(dem,wImg,imgPos.x,imgPos.y,cx,cy);
}
*/

void WheelModel::SetupWheel(const ProcConfig& pc, const WheelConfig &conf)
{
    config_ = conf;

    WheelRender wr;

    wr.groundLevel = pc.wheelGroundLevel;
    wr.heightScale = pc.heightScale;
    wr.maxHeight = pc.maxHeight;
    wr.pixelSize = pc.pixelSize;


    this->jointPosImg_ = config_.jointPosWheel*(1.0/pc.pixelSize);
    this->radiusImg_ = config_.radius/pc.pixelSize;
    this->widthImg_ = config_.width/pc.pixelSize;
    this->latRadiusImg_ = config_.latRadius/pc.pixelSize;
    this->wheelSupportThreshold_ = (int)(config_.radius*pc.heightScale*pc.wheelSupportThresholdFactor);


    angleStep_ = (CV_PI*2.0)/(double)pc.numAngleStep;


    descriptors_.clear();


    for (int tl = 0; tl < pc.numAngleStep;++tl)
    {
        WheelDescriptor desc = wr.RenderWheelDesc(config_.radius,config_.width,config_.latRadius,angleStep_*(float)tl, config_.jointPosWheel);

        descriptors_.push_back(desc);





    }



}


