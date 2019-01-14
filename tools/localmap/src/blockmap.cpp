
#include "blockmap.h"
#include <opencv2/imgproc/imgproc.hpp>
#include "utils_depth_image.h"

void BlockMap::Setup()
{
    blockResolution_ = mapResolution_ / numBlocks_;
    safeBlocks_ = 2;//numBlocks_/2;
    blockStep_ = mapSize_/(float) numBlocks_;
    safeMin_.x = (float)((numBlocks_-safeBlocks_)/2)*blockStep_;
    safeMin_.y = (float)((numBlocks_-safeBlocks_)/2)*blockStep_;
    safeMax_.x = (float)(numBlocks_-(numBlocks_-safeBlocks_)/2)*blockStep_;
    safeMax_.y = (float)(numBlocks_-(numBlocks_-safeBlocks_)/2)*blockStep_;

    currentMap_ = cv::Mat(mapResolution_,mapResolution_,CV_32F);
    tempMap_ = cv::Mat(mapResolution_,mapResolution_,CV_32F);

    UpdateCenter(cv::Point2f(0,0));

    curPos_ = cv::Point3f(0,0,0);
    curNormal_ = cv::Point3f(0,0,1);


}
void BlockMap::SetPose(cv::Point3f normal, cv::Point3f pos)
{
    curNormal_ = normal;
    curPos_ = pos;
}

void BlockMap::SetSafeBlocksTo()
{
    //const float heightPixelRatio = heightScale/pixelSizeInv;
    const float heightPixelRatio = heightScale_/pixelResolution_;
    const float absZInv1 = curNormal_.z == 0 ? 0 : 1.0f/std::abs(curNormal_.z);

    const float dx1 = (-curNormal_.x*absZInv1)*heightPixelRatio;
    const float dy1 = ( curNormal_.y*absZInv1)*heightPixelRatio;


    cv::Point2i safeBlockOrigin((numBlocks_-safeBlocks_)/2,(numBlocks_-safeBlocks_)/2);
    cv::Point2i safeBlockSize(safeBlocks_,safeBlocks_);

    cv::Rect drawRect(safeBlockOrigin.x*blockResolution_,safeBlockOrigin.y*blockResolution_,safeBlockSize.x*blockResolution_,safeBlockSize.y*blockResolution_);

    cv::Point2f mapPos = RobotPos2MapPos(cv::Point2f(curPos_.x,curPos_.y));

    cv::Point2f wcPos1( (mapPos.x)-drawRect.x, (mapPos.y)-drawRect.y);

    float startVal = (curPos_.z*heightScale_)- wcPos1.x*dx1 - wcPos1.y*dy1;

    startVal += mapBaseLevel_;
    cv::Mat tmat = currentMap_(drawRect);

    float *mapPtr;

    for (int y = 0; y < tmat.rows;++y)
    {
        mapPtr = tmat.ptr<float>(y);
        for (int x = 0; x < tmat.cols;++x)
        {
            mapPtr[x] = (startVal + dx1*(float)x + dy1*(float)y);
        }
    }
}

void BlockMap::SetSafeAroundRobot()
{
    //const float heightPixelRatio = heightScale/pixelSizeInv;
    const float heightPixelRatio = heightScale_/pixelResolution_;
    const float absZInv1 = curNormal_.z == 0 ? 0 : 1.0f/std::abs(curNormal_.z);

    const float dx1 = (-curNormal_.x*absZInv1)*heightPixelRatio;
    const float dy1 = ( curNormal_.y*absZInv1)*heightPixelRatio;

    cv::Point2f mapPos = RobotPos2MapPos(cv::Point2f(curPos_.x,curPos_.y));

    cv::Point2i safeBlockSize(safeBlocks_,safeBlocks_);

    cv::Rect drawRect(mapPos.x - safeBlockSize.x*blockResolution_*0.5 ,mapPos.y - safeBlockSize.y*blockResolution_*0.5 ,safeBlockSize.x*blockResolution_,safeBlockSize.y*blockResolution_);


    cv::Point2f wcPos1( (mapPos.x)-drawRect.x, (mapPos.y)-drawRect.y);

    float startVal = (curPos_.z*heightScale_)- wcPos1.x*dx1 - wcPos1.y*dy1;

    startVal += mapBaseLevel_;
    cv::Mat tmat = currentMap_(drawRect);

    float *mapPtr;

    for (int y = 0; y < tmat.rows;++y)
    {
        mapPtr = tmat.ptr<float>(y);
        for (int x = 0; x < tmat.cols;++x)
        {
            mapPtr[x] = (startVal + dx1*(float)x + dy1*(float)y);
        }
    }
}


/*
void BlockMap::SetSafeBlocksTo(float val)
{
    cv::Point2i safeBlockOrigin((numBlocks_-safeBlocks_)/2,(numBlocks_-safeBlocks_)/2);
    cv::Point2i safeBlockSize(safeBlocks_,safeBlocks_);

    cv::Rect drawRect(safeBlockOrigin.x*blockResolution_,safeBlockOrigin.y*blockResolution_,safeBlockSize.x*blockResolution_,safeBlockSize.y*blockResolution_);

    cv::Mat tmat = currentMap_(drawRect);
    tmat.setTo(val);

}


void BlockMap::SetSafeBlocksTo(const cv::Point3f &normal, const cv::Point3f &pos, const float &heightScale)
{
    //const float heightPixelRatio = heightScale/pixelSizeInv;
    const float heightPixelRatio = heightScale/pixelResolution_;
    const float absZInv1 = normal.z == 0 ? 0 : 1.0f/std::abs(normal.z);

    const float dx1 = (-normal.x*absZInv1)*heightPixelRatio;
    const float dy1 = ( normal.y*absZInv1)*heightPixelRatio;


    cv::Point2i safeBlockOrigin((numBlocks_-safeBlocks_)/2,(numBlocks_-safeBlocks_)/2);
    cv::Point2i safeBlockSize(safeBlocks_,safeBlocks_);

    cv::Rect drawRect(safeBlockOrigin.x*blockResolution_,safeBlockOrigin.y*blockResolution_,safeBlockSize.x*blockResolution_,safeBlockSize.y*blockResolution_);

    cv::Point2f wcPos1( (pos.x*pixelResolution_)-drawRect.x, (pos.y*pixelResolution_)-drawRect.y);

    float startVal = (pos.z*heightScale)- wcPos1.x*dx1 - wcPos1.y*dy1;

    startVal += mapBaseLevel_;
    cv::Mat tmat = currentMap_(drawRect);

    float *mapPtr;

    for (int y = 0; y < tmat.rows;++y)
    {
        mapPtr = tmat.ptr<float>(y);
        for (int x = 0; x < tmat.cols;++x)
        {
            mapPtr[x] = (startVal + dx1*(float)x + dy1*(float)y);
        }
    }


}
*/


void BlockMap::SetMapTo(float val)
{
    currentMap_.setTo(val);

}



void BlockMap::UpdateCenter(const cv::Point2f nCenter)
{
    center_ = nCenter;
    origin_.x = center_.x - mapSize_/2.0;
    origin_.y = center_.y - mapSize_/2.0;

    safeMin_.x = center_.x- ((float)safeBlocks_/2.0f)*blockStep_;
    safeMin_.y = center_.y- ((float)safeBlocks_/2.0f)*blockStep_;
    safeMax_.x = center_.x+ ((float)safeBlocks_/2.0f)*blockStep_;
    safeMax_.y = center_.y+ ((float)safeBlocks_/2.0f)*blockStep_;



}

cv::Point2d BlockMap::RobotPos2MapPos(cv::Point2d pos)
{
    cv::Point2d mapPos = pos-cv::Point2d(origin_.x, origin_.y);

    cv::Point2d pixelPos = mapPos*pixelResolution_;

    return pixelPos;
}
cv::Point2f BlockMap::RobotPos2MapPos(cv::Point2f pos)
{
    cv::Point2f mapPos = pos-origin_;

    cv::Point2f pixelPos = mapPos*pixelResolution_;

    return pixelPos;
}

void BlockMap::Transform2BaseLink(const cv::Point2d &pos, const double angle)
{
    cv::Point2d mapPos = RobotPos2MapPos(pos);
    cv::Mat rot = cv::getRotationMatrix2D(mapPos,angle*(180.0/CV_PI),1.0);
    cv::Mat row = cv::Mat::zeros(1,3,CV_64F);
    row.at<double>(0,2) = 1.0;
    rot.push_back(row);

    cv::Point2d transVec(((double)mapResolution_/2.0)-mapPos.x,((double)mapResolution_/2.0)-mapPos.y);

    cv::Mat trans = cv::Mat::eye(3,3,CV_64F);
    trans.at<double>(0,2) = transVec.x;
    trans.at<double>(1,2) = transVec.y;

    cv::Mat warpMat = trans*rot;
    cv::Mat affineMat = warpMat.rowRange(0,2);

    cv::warpAffine(currentMap_,baseLinkMap_,affineMat,currentMap_.size(),CV_INTER_NN);

    //return baseLinkMap_;
}


bool BlockMap::TestSafe(const cv::Point2f &pos)
{
    cv::Point2f tsafeMin = safeMin_;
    cv::Point2f tsafeMax = safeMax_;

    return !(pos.x < tsafeMin.x || pos.y < tsafeMin.y || pos.x > tsafeMax.x || pos.y > tsafeMax.y);
}

cv::Point2i BlockMap::GetClosestBlockPos(const cv::Point2f &pos)
{
    cv::Point2f mapPos = pos-origin_;
    cv::Point2i blockIdx;
    blockIdx.x = round(mapPos.x / blockStep_);
    blockIdx.y = round(mapPos.y / blockStep_);
    return blockIdx;

}


void BlockMap::CenterMap(const cv::Point2i &newCenterBlock)
{
    cv::Point2i centerBlock(numBlocks_/2,numBlocks_/2);

    cv::Point2i shiftBlocks = newCenterBlock-centerBlock;

    cv::Point2i shiftPixels = shiftBlocks*blockResolution_;

    cv::Point2i dimensions(mapResolution_,mapResolution_);
    cv::Point2i resMin = shiftPixels;

    if (resMin.x < 0) resMin.x = 0;
    if (resMin.y < 0) resMin.y = 0;

    cv::Point2i resMax = shiftPixels+dimensions;

    if (resMax.x > mapResolution_) resMax.x = mapResolution_;
    if (resMax.y > mapResolution_) resMax.y = mapResolution_;

    int width = resMax.x-resMin.x;
    int height = resMax.y-resMin.y;

    if (width <= 0 || height <= 0)
    {
        UtilsDepthImage::SetToZero(currentMap_);
        SetSafeBlocksTo();
        //currentMap_.setTo(0);
        //return;
    }
    else
    {

        cv::Rect curMapRect(resMin.x, resMin.y,width, height);
        cv::Rect targetMapRect(resMin.x-shiftPixels.x, resMin.y-shiftPixels.y,width, height);

        UtilsDepthImage::SetToZero(tempMap_);

        cv::Mat curImg = currentMap_(curMapRect);
        cv::Mat targetImg = tempMap_(targetMapRect);

        curImg.copyTo(targetImg);

        tempMap_.copyTo(currentMap_);



    }
    cv::Point2f newCenter;
    newCenter = cv::Point2f(center_.x,center_.y) + cv::Point2f(shiftBlocks.x,shiftBlocks.y)*blockStep_;
    UpdateCenter(newCenter);

    if (abs(shiftBlocks.x) >= safeBlocks_ || abs(shiftBlocks.y) >= safeBlocks_ )
    {
        SetSafeBlocksTo();

    }


}

void BlockMap::ReCenter(const cv::Point2f &pos)
{
    if (!TestSafe(pos))
    {
        cv::Point2i closestBlockPos = GetClosestBlockPos(pos);
        CenterMap(closestBlockPos);

        lastPosition_ = pos;

    }


}
