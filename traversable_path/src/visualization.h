#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "pointclassification.h"

class Visualization
{
public:
    Visualization();

    void paintPath(std::vector<PointClassification>);

private:
    cv::Mat path_;
    bool is_path_initialized_;
    unsigned int path_pos_;
};

#endif // VISUALIZATION_H
