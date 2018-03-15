#include "wheelrender.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

WheelRender::WheelRender()
{

}

void WheelRender::ConvertImg(CVAlignedMat &img, const CVAlignedMat &mask)
{

    for (int y = 0;y < mask.mat_.rows;y++)
    {
        const short* maskPtr = mask.mat_.ptr<short>(y);
        short* imgPtr = img.mat_.ptr<short>(y);

        for (unsigned int x = 0;x < mask.mat_.step.buf[0]/2;x++)
        {
            if (maskPtr[x] == 0) imgPtr[x] = maxHeight;

        }
    }

}


WheelDescriptor WheelRender::CreateDescriptor(cv::Mat wheelImageF, cv::Mat wheelMask, cv::Point2f centerPos, cv::Vec2f dirX, cv::Vec2f dirY)
{
    cv::Mat wheelImg;

    wheelImageF.convertTo(wheelImg,CV_16S,heightScale,groundLevel);



    /// Debug
    /*
    cv::Mat displayMat(wheelImg.rows,wheelImg.cols,CV_8U);

    wheelImg.convertTo(displayMat,CV_8U,1.0,-groundLevel);

    cv::namedWindow("Wheel",0);
    cv::imshow("Wheel",displayMat);

    cv::waitKey();
    */
    /// End Debug

    double minV,maxV;
    cv::minMaxLoc(wheelImg,&minV,&maxV);

    cv::Point2f newCenter;

    cv::Mat resImg = CropTemplate<short>(wheelImg,centerPos,newCenter,groundLevel);
    cv::Mat resMask = CropTemplate<short>(wheelMask,centerPos,newCenter);

    /*
    cv::Mat displayMat(resImg.rows,resImg.cols,CV_8U);
    resImg.convertTo(displayMat,CV_8U,0.5,-groundLevel/2.0);
    cv::namedWindow("Wheel",0);
    cv::imshow("Wheel",displayMat);
    cv::waitKey();
    */
    WheelDescriptor desc;

    CVAlignedMat::ptr aMask = CVAlignedMat::Create(resMask);

    desc.image_ = CVAlignedMat::Create(resImg);

    ConvertImg(*desc.image_,*aMask);


    desc.centerImg_ = newCenter;
    desc.dirX_.x = dirX[0];
    desc.dirX_.y = dirX[1];
    desc.dirY_.x = dirY[0];
    desc.dirY_.y = dirY[1];

    return desc;


}

WheelDescriptor WheelRender::RenderWheelDesc(float radius, float width, float latRadius,float angle, cv::Point2f jointPos)
{
    WheelDescriptor desc;
    if (latRadius == 0) desc = RenderWheelDescCyl(radius,width,angle);
    else desc = RenderWheelDescSph(radius,width,latRadius,angle);

    desc.numImagePixels_ = 0;
    for (int y = 0; y < desc.image_->mat_.rows;++y)
    {
        for (int x = 0; x < desc.image_->mat_.cols;++x)
        {
            if (desc.image_->mat_.at<short>(y,x) <  maxHeight) desc.numImagePixels_++;

        }

    }

    desc.numImagePixelsInv_ = 1.0f/(float)desc.numImagePixels_;

    //desc.jointPosImg_ = (desc.centerImg_ + ((desc.dirX_*jointPos.x)/pixelSize) + ((desc.dirY_*jointPos.y)/pixelSize));
    desc.jointPosImg_ = (desc.centerImg_ + ((desc.dirX_*jointPos.x)*(1.0/pixelSize)) + ((desc.dirY_*jointPos.y)*(1.0/pixelSize)));

    return desc;
}


WheelDescriptor WheelRender::RenderWheelDescCyl(float radius, float width,float angle)
{
    float pixelRad = radius/pixelSize;
    float pixelWidth = pixelRad*2.0;
    float pixelHeight = (width)/pixelSize;

    int pixelWidthI = round(pixelWidth);
    int pixelHeightI = round(pixelHeight);


    float xdir = cos(angle);
    float ydir = sin(angle);

    float xdirO = -ydir;
    float ydirO = xdir;

    int imageSizeC = pixelWidth+pixelHeight;


    cv::Mat wheelImageF(imageSizeC,imageSizeC,CV_32F);
    cv::Mat wheelMask(imageSizeC,imageSizeC,CV_16S);
    wheelMask.setTo(1);

    cv::Point2f centerPos;

    centerPos.x = imageSizeC/2;
    centerPos.y = imageSizeC/2;


    if (pixelWidthI%2 == 1)
    {
        centerPos.x += 0.5f;
    }
    if (pixelHeightI%2 == 1)
    {
        centerPos.y += 0.5f;
    }


    cv::Point2f pixelPos;
    cv::Vec2f wheelDir,wheelDirO;

    wheelDir[0] = xdir;
    wheelDir[1] = ydir;
    wheelDirO[0] = xdirO;
    wheelDirO[1] = ydirO;

    wheelDir = cv::normalize(wheelDir);
    wheelDirO = cv::normalize(wheelDirO);


    cv::Vec2f pixDir;




    for (int r = 0; r < wheelImageF.rows;++r)
    {
        for (int c = 0; c < wheelImageF.cols;++c)
        {
            pixelPos.x = ((float)c)+0.5f;
            pixelPos.y = ((float)r)+0.5f;

            pixDir = pixelPos- centerPos;

            cv::Vec2f tvec = Cross2D(wheelDir,pixDir);
            cv::Vec2f tvecO = Cross2D(wheelDirO,pixDir);

            float dist = cv::norm(tvecO);
            float distO = cv::norm(tvec);


            float aval = std::abs( dist )/pixelRad;
            if (aval < 1.0)
            {
                float yVal = -sqrt(1.0- pow(aval,2.0) )+1.0;

                if (distO > pixelHeight/2.0)
                {
                    wheelImageF.at<float>(r,c) = 0.0f;
                    wheelMask.at<short>(r,c) = 0;
                }
                else wheelImageF.at<float>(r,c) = yVal*radius;

            }
            else
            {
                wheelImageF.at<float>(r,c) = 0.0f;
                wheelMask.at<short>(r,c) = 0;
            }





        }

    }

    double minV,maxV;
    cv::minMaxLoc(wheelImageF,&minV,&maxV);

    /*
    cv::Mat displayMat(wheelImageF.rows,wheelImageF.cols,CV_8U);

    wheelImageF.convertTo(displayMat,CV_8U,255.0,0);

    cv::namedWindow("Wheel",0);
    cv::imshow("Wheel",displayMat);

    cv::waitKey();
    */
    return CreateDescriptor(wheelImageF,wheelMask,centerPos,wheelDir,wheelDirO);


}

double WheelRender::GetEllipsePointZ(double x, double y, double a, double b, double c)
{
    double x2Oa2 = (x*x)/(a*a);
    double y2Ob2 = (y*y)/(b*b);
    double c2 = c*c;

    if (x2Oa2+y2Ob2 >= 1.0) return 0.0;

    return sqrt(((1.0 - x2Oa2-y2Ob2)*c2));
}


WheelDescriptor WheelRender::RenderWheelDescSph(float radius, float width, float latRadius,float angle)
{

    float pixelRad = radius/pixelSize;
    float pixelLatRadius = latRadius/pixelSize;
    float pixelWidth = pixelRad*2.0;
    float pixelHeight = (width)/pixelSize;

    int pixelWidthI = round(pixelWidth);
    int pixelHeightI = round(pixelHeight);

    int imageSizeC = pixelWidth+pixelHeight;


    cv::Mat wheelImageF(imageSizeC,imageSizeC,CV_32F);
    cv::Mat wheelMask(imageSizeC,imageSizeC,CV_16S);
    wheelMask.setTo(1);

    cv::Point2f centerPos;

    centerPos.x = imageSizeC/2;
    centerPos.y = imageSizeC/2;


    if (pixelWidthI%2 == 1)
    {
        centerPos.x += 0.5f;
    }
    if (pixelHeightI%2 == 1)
    {
        centerPos.y += 0.5f;
    }


    cv::Point2f pixelPos;
    cv::Vec2f wheelDir,wheelDirO;

    float xdir = cos(angle);
    float ydir = sin(angle);

    float xdirO = -ydir;
    float ydirO = xdir;

    wheelDir[0] = xdir;
    wheelDir[1] = ydir;
    wheelDirO[0] = xdirO;
    wheelDirO[1] = ydirO;

    wheelDir = cv::normalize(wheelDir);
    wheelDirO = cv::normalize(wheelDirO);


    cv::Vec2f pixDir;




    for (int r = 0; r < wheelImageF.rows;++r)
    {
        for (int c = 0; c < wheelImageF.cols;++c)
        {
            pixelPos.x = ((float)c)+0.5f;
            pixelPos.y = ((float)r)+0.5f;

            pixDir = pixelPos- centerPos;

            cv::Vec2f tvec = Cross2D(wheelDir,pixDir);
            float distO = cv::norm(tvec);



            float px = wheelDir.dot(pixDir);
            float py = wheelDirO.dot(pixDir);

            float zVal = GetEllipsePointZ(px,py,pixelRad,pixelLatRadius,pixelRad);

            if (distO > pixelHeight/2.0) zVal = 0;

            if (zVal > 0.0)
            {

                float mLatVal = pixelRad-zVal;
                wheelImageF.at<float>(r,c) = mLatVal/pixelRad;

            }
            else
            {
                wheelImageF.at<float>(r,c) = 0.0f;
                wheelMask.at<short>(r,c) = 0;
            }





        }

    }

    wheelImageF*= radius;

    double minV,maxV;
    cv::minMaxLoc(wheelImageF,&minV,&maxV);


    /*
    cv::Mat displayMat(wheelImageF.rows,wheelImageF.cols,CV_8U);

    wheelImageF.convertTo(displayMat,CV_8U,2550.0,0);

    cv::namedWindow("Wheel",0);
    cv::imshow("Wheel",displayMat);

    cv::waitKey();
    */
    return CreateDescriptor(wheelImageF,wheelMask,centerPos,wheelDir,wheelDirO);

}
