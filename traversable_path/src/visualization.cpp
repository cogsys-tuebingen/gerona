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

    bool traversable = false;
    int last_toggle = 0;
    for (unsigned int i = 1; i < points.size(); ++i) { //TODO: gedanken über i=1 machen :P
        if ( (points[i].traversable_by_intensity && points[i].traversable_by_range) != traversable ) {
            cv::line(path_, cv::Point(path_pos_, last_toggle), cv::Point(path_pos_, i-1),
                     traversable ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255));

            traversable = !traversable;
            last_toggle = i;
        }
    }

    // last segment
    cv::line(path_, cv::Point(path_pos_, last_toggle), cv::Point(path_pos_, path_.rows-1),
             traversable ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255));

    cv::imshow("path", path_);
    cv::waitKey(3);

    path_pos_ = ++path_pos_ % path_.cols;
}
