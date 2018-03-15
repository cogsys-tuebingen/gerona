#include "scaleddrawproc.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


ScaledDrawProc::ScaledDrawProc()
{
    scaleFactor_ = 8;
}

void ScaledDrawProc::SetMat(cv::Mat &image)
{
    cv::resize(image,image_,cv::Size(image.cols*scaleFactor_,image.rows*scaleFactor_),scaleFactor_,scaleFactor_,CV_INTER_NN);

}

void ScaledDrawProc::DrawImage(cv::Mat &image, cv::Point2f pos)
{
    cv::Mat tImg;
    cv::resize(image,tImg,cv::Size(image.cols*scaleFactor_,image.rows*scaleFactor_),scaleFactor_,scaleFactor_,CV_INTER_NN);

    cv::Rect trect(pos*scaleFactor_,cv::Size(tImg.cols,tImg.rows));
    cv::Mat curImg = image_(trect);
    tImg.copyTo(curImg);

}

void ScaledDrawProc::DrawImage(cv::Mat &image, cv::Mat &mask, cv::Point2f pos)
{
    cv::Mat tImg;
    cv::resize(image,tImg,cv::Size(image.cols*scaleFactor_,image.rows*scaleFactor_),scaleFactor_,scaleFactor_,CV_INTER_NN);

    cv::Mat tMask;
    cv::resize(mask,tMask,cv::Size(mask.cols*scaleFactor_,mask.rows*scaleFactor_),scaleFactor_,scaleFactor_,CV_INTER_NN);


    cv::Rect trect(pos*scaleFactor_,cv::Size(tImg.cols,tImg.rows));

    if (trect.x < 0 || trect.y < 0 || trect.br().x >= image_.cols || trect.br().y >= image_.rows) return;

    cv::Mat curImg = image_(trect);
    tImg.copyTo(curImg,tMask);

}

void ScaledDrawProc::DrawLineScaled(cv::Point2f p1, cv::Point2f p2, cv::Scalar color)
{
    cv::line(image_,p1*scaleFactor_,p2*scaleFactor_,color);
}

void ScaledDrawProc::DrawLineScaled(cv::Point2f p1, cv::Point2f p2, cv::Scalar color, int thickness)
{
    cv::line(image_,p1*scaleFactor_,p2*scaleFactor_,color,thickness);
}

void ScaledDrawProc::DrawCrossScaled(cv::Point2f p1, float size, cv::Scalar color)
{
    cv::Point2f px = p1 + cv::Point2f(size,0);
    cv::Point2f py = p1 + cv::Point2f(0,size);
    cv::Point2f mx = p1 - cv::Point2f(size,0);
    cv::Point2f my = p1 - cv::Point2f(0,size);

    cv::line(image_,px*scaleFactor_,mx*scaleFactor_,color);
    cv::line(image_,py*scaleFactor_,my*scaleFactor_,color);
}

void ScaledDrawProc::DrawArrowScaled(cv::Point2f p1, float angle, float length, float radius, cv::Scalar color)
{
    float ay = sin(angle);
    float ax = cos(angle);

    cv::Point2f px = p1;
    cv::Point2f mx = p1 + cv::Point2f(ax,ay)*length;

    cv::line(image_,px*scaleFactor_,mx*scaleFactor_,color);

    cv::circle(image_,p1*scaleFactor_,radius,color,-1);

}


void ScaledDrawProc::DrawCircleScaled(cv::Point2f p1, float size, cv::Scalar color)
{

    cv::circle(image_,p1*scaleFactor_,size*scaleFactor_,color);
}
void ScaledDrawProc::DrawCircleScaled(cv::Point2f p1, float size, cv::Scalar color, int thickness)
{

    cv::circle(image_,p1*scaleFactor_,size*scaleFactor_,color,thickness);
}
void ScaledDrawProc::PutText(cv::Point2f p1, std::string text, cv::Scalar color)
{

    cv::putText(image_,text,p1*scaleFactor_,CV_FONT_HERSHEY_COMPLEX,1.0,color);
}


