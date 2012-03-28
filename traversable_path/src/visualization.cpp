#include "visualization.h"

using namespace std;

Visualization::Visualization()
{
    is_path_initialized_ = false;
    cv::namedWindow("path", CV_WINDOW_FREERATIO);
}

void Visualization::paintPath(std::vector<PointClassification> points)
{
    if (!is_path_initialized_) {
        path_ = cv::Mat(cv::Size(1000, points.size()), CV_8UC3);
        path_pos_ = 0;
        is_path_initialized_ = true;
    }

    PointClassification current_state = points[0];
    int last_toggle = 0;
    unsigned int points_size = points.size();
    for (unsigned int i = 1; i < points_size; ++i) {
        if (points[i] != current_state || i == points_size-1) {
            cv::Scalar color;
            if (current_state.traversable_by_intensity && current_state.traversable_by_range) {
                color = cv::Scalar(0, 255, 0); // green
            } else if (current_state.traversable_by_intensity) {
                color = cv::Scalar(255, 0, 0); // blue
            } else if (current_state.traversable_by_range) {
                color = cv::Scalar(0, 255, 255); // yellow
            } else {
                color = cv::Scalar(0, 0, 255); // red
            }

            cv::line(path_, cv::Point(path_pos_, last_toggle), cv::Point(path_pos_, i-1), color);

            current_state = points[i];
            last_toggle = i;
        }
    }

    cv::imshow("path", path_);
    cv::waitKey(3);

    path_pos_ = ++path_pos_ % path_.cols;
}
