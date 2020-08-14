#include "chassismodel.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>

#include "wheelrender.h"

//#include "scaleddrawproc.h"
#include "utils_diff.h"


ChassisModel::ChassisModel()
{

}

void ChassisModel::SetupChassis(const ProcConfig& procConfig, const ChassisConfig &conf)
{

    if (!conf.testChassis) return;

    config_ = conf;

    cv::Mat orgImg = cv::imread(conf.chassisfileName,-1);

    if (orgImg.channels() == 3)
    {
        cv::cvtColor(orgImg,orgImg,cv::COLOR_BGR2GRAY);
    }
    if (orgImg.channels() == 4)
    {
        cv::cvtColor(orgImg,orgImg,cv::COLOR_BGRA2GRAY);
    }

    cv::Mat imgConvF;
    orgImg.convertTo(imgConvF,CV_32F);

    imgConvF *= conf.chassisImageValueScale;
    imgConvF += conf.chassisImageValueOffset;

    float mincv = 9999999.0f,maxcv = -9999999.0f;

    for (int y = 0; y < imgConvF.rows;++y)
    {
        for (int x = 0; x < imgConvF.cols;++x)
        {
            if (imgConvF.at<float>(y,x) == conf.chassisImageValueOffset)
            {
                imgConvF.at<float>(y,x) = 0;
            }
            else
            {
                if (imgConvF.at<float>(y,x) < mincv) mincv = imgConvF.at<float>(y,x);
                if (imgConvF.at<float>(y,x) > maxcv) maxcv = imgConvF.at<float>(y,x);
            }

        }
    }

    // Debug Output
    /*
    cv::Mat colorImg;

    imgConvF.convertTo(colorImg,CV_8U,1.0,0);

    cv::cvtColor(colorImg,colorImg,cv::COLOR_GRAY2BGR);

    ScaledDrawProc sd;

    sd.SetScaleFactor(8);
    sd.SetMat(colorImg);

    //sd.DrawCrossScaled(cropCenter,3,cv::Scalar(255,0,0));



    cv::namedWindow("chassis",0);
    cv::imshow("chassis",sd.GetImage());

    cv::waitKey();
    */
    // Debug Output

    cv::Mat imgConv;
    imgConvF.convertTo(imgConv,CV_16S);




    cv::Point2f chassisImageCenter = conf.chassisImageCenter;

    if (conf.chassisImageCenter.x == -1.0 && conf.chassisImageCenter.y == -1.0)
    {
        chassisImageCenter.x = ((double)imgConvF.cols)/2.0;
        chassisImageCenter.y = ((double)imgConvF.rows)/2.0;

    }


    float chassisHeight = (float)orgImg.rows * procConfig.pixelSize;

    float scaleFact = conf.chassisModelYSize/chassisHeight;


    cv::resize(imgConv,orgImg,cv::Size((float)orgImg.cols*scaleFact,(float)orgImg.rows*scaleFact),scaleFact,scaleFact,CV_INTER_NN);

    cv::Point2f scaledCenter = chassisImageCenter*scaleFact;

    descriptors_.clear();


    WheelRender wr;

    wr.groundLevel = procConfig.wheelGroundLevel;
    wr.heightScale = procConfig.heightScale;
    wr.maxHeight = procConfig.maxHeight;
    wr.pixelSize = procConfig.pixelSize;



    int imgSize = orgImg.rows + orgImg.cols;

    imgSize *= 2;


    int imgSizeDRows = imgSize- orgImg.rows;
    int imgSizeDCols = imgSize- orgImg.cols;

    int nrow = imgSizeDRows/2;
    int ncol = imgSizeDCols/2;


    cv::Rect trect(ncol,nrow,orgImg.cols,orgImg.rows);

    cv::Mat chassisImage(imgSize,imgSize,CV_16S);
    chassisImage.setTo(0);

    cv::Mat tImage = chassisImage(trect);

    orgImg.copyTo(tImage);


    cv::Point2f newCenter = scaledCenter + cv::Point2f(ncol,nrow);



    for (int tl = 0; tl < procConfig.numAngleStep;tl++)
    {
        ChassisDescriptor desc;

        float angle = (float)tl*procConfig.angleStep;

        cv::Mat rotMat = cv::getRotationMatrix2D(newCenter,-angle*(180.0/CV_PI),1.0);

        cv::Mat rotImage;

        cv::warpAffine(chassisImage,rotImage,rotMat,cv::Size(imgSize,imgSize),CV_INTER_NN);


        cv::Point2f cropCenter;
        cv::Mat cropped = wr.CropTemplate<short>(rotImage,newCenter,cropCenter);

        cv::Mat resImg = cropped += procConfig.wheelGroundLevel;

        cv::Mat mask(resImg.rows,resImg.cols,resImg.type());
        mask.setTo(0);

        for (int y = 0; y < cropped.rows;++y)
        {
            for (int x = 0; x < cropped.cols;++x)
            {
                if (resImg.at<short>(y,x) != procConfig.wheelGroundLevel) mask.at<short>(y,x) = 1;
            }
        }


        desc.image_ = CVAlignedMat::Create(resImg);
        CVAlignedMat::ptr maskA = CVAlignedMat::Create(mask);

        wr.ConvertImg(*desc.image_.get(),*maskA.get());

        desc.centerImg_ = cropCenter;

        descriptors_.push_back(desc);


        /*
        cv::Mat colorImg;

        cropped.convertTo(colorImg,CV_8U,1.0/256.0,-procConfig.mapBaseHeight/256.0);

        cv::cvtColor(colorImg,colorImg,cv::COLOR_GRAY2BGR);

        ScaledDrawProc sd;

        sd.SetScaleFactor(8);
        sd.SetMat(colorImg);

        sd.DrawCrossScaled(cropCenter,3,cv::Scalar(255,0,0));



        cv::namedWindow("chassis",0);
        cv::imshow("chassis",sd.GetImage());

        cv::waitKey();
        */


    }

}


int ChassisModel::Evaluate(const cv::Mat &dem,const float &startVal, const float &dx, const float &dy, const cv::Point2f &pos, const int &angleIdx, int &cx, int &cy) const
{
    ChassisDescriptor desc = GetDescriptorIdx(angleIdx);

    cv::Point2i tPos(round(pos.x-desc.centerImg_.x),round(pos.y-desc.centerImg_.y));

    return Utils_DIFF::testChassis(dem,desc.image_->mat_,startVal,dx,dy,tPos.x,tPos.y,cx,cy);




}


int ChassisModel::EvaluateNP(const cv::Mat &dem,const float &startVal, const float &dx, const float &dy, const cv::Point2f &pos, const int &angleIdx, int &cx, int &cy) const
{
    cx = 0;
    cy = 0;
    ChassisDescriptor desc = GetDescriptorIdx(angleIdx);

    cv::Point2i tPos(round(pos.x-desc.centerImg_.x),round(pos.y-desc.centerImg_.y));

    return Utils_DIFF::np_testChassis(dem,desc.image_->mat_,startVal,dx,dy,tPos.x,tPos.y,cx,cy);




}

