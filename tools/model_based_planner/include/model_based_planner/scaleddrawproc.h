#ifndef SCALEDDRAWPROC_H
#define SCALEDDRAWPROC_H


#include <opencv2/core/core.hpp>

/**
 * @brief Helper class for drawing debug images
 */
class ScaledDrawProc
{
public:
    ScaledDrawProc();

    void SetMatNoScale(cv::Mat &image);

    void SetMat(cv::Mat &image);

    void SetScaleFactor(float factor) {scaleFactor_ = factor;}

    void DrawImage(cv::Mat &image, cv::Point2f pos);
    void DrawImage(cv::Mat &image, cv::Mat &mask, cv::Point2f pos);

    void DrawLineScaled(cv::Point2f p1, cv::Point2f p2, cv::Scalar color);
    void DrawLineScaled(cv::Point2f p1, cv::Point2f p2, cv::Scalar color, int thickness);

    void DrawCrossScaled(cv::Point2f p1, float size, cv::Scalar color);

    void DrawCircleScaled(cv::Point2f p1, float size, cv::Scalar color);
    void DrawCircleScaled(cv::Point2f p1, float size, cv::Scalar color, int thickness);

    cv::Mat GetImage() {return image_;}

    void PutText(cv::Point2f p1, std::string text, cv::Scalar color);

    void DrawArrowScaled(cv::Point2f p1, float angle, float length, float radius, cv::Scalar color);


private:
    cv::Mat image_;
    float scaleFactor_;
};

#endif // SCALEDDRAWPROC_H
