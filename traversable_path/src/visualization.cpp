#include "visualization.h"
#include <opencv2/highgui/highgui.hpp>

using namespace std;

Visualization::Visualization()
{
    is_path_initialized_ = false;
    cv::namedWindow("path", CV_WINDOW_FREERATIO);
}


void Visualization::paintPath(const pcl::PointCloud<PointXYZRGBT>::ConstPtr &cloud)
{
    if (cloud->points.empty()) {
        ROS_ERROR("Visualization::paintPath: Point array is empty.");
        return;
    }

    if (!is_path_initialized_) {
        ROS_INFO("Initialize path image. Size: 1000x%zu", cloud->points.size());
        path_ = cv::Mat(cv::Size(1000, cloud->points.size()), CV_8UC3);
        path_pos_ = 0;
        is_path_initialized_ = true;
    }

    // blackline before the current one to make the current position more visible
    int black_line_pos = (path_pos_ + 1) % path_.cols;
    cv::line(path_, cv::Point(black_line_pos, 0), cv::Point(black_line_pos, path_.rows-1), cv::Scalar(0,0,0));


    bool current_state = cloud->points[0].traversable;
    int last_toggle = 0;
    cv::Scalar color;
    for (int i = 1; i < path_.rows; ++i) {
        if (cloud->points[i].traversable != current_state) {
            color = current_state ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
            cv::line(path_, cv::Point(path_pos_, last_toggle), cv::Point(path_pos_, i-1), color);

            current_state = cloud->points[i].traversable;
            last_toggle = i;
        }
    }
    // the last segment
    color = current_state ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
    cv::line(path_, cv::Point(path_pos_, last_toggle), cv::Point(path_pos_, path_.rows-1), color);


    cv::imshow("path", path_);
    cv::waitKey(3);

    path_pos_ = ++path_pos_ % path_.cols;
}
