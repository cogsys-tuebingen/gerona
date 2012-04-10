#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <ros/ros.h>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "pointclassification.h"

/**
 * @brief Provides methods to visualize the terrain classification.
 *
 * @author Felix Widmaier
 * @version $Id$
 */
class Visualization
{
public:
    Visualization();

    /**
     * @brief Paint the next line to the path image.
     *
     * The image is initialized at the first call of this method. Following calls will add lines from left to right,
     * starting again from left, when the right end of the image is reached.
     *
     * @param The terrain classification for each point of the current laser scan.
     */
    void paintPath(std::vector<PointClassification>);

    // do the same with a bool-vector
    void paintPath(std::vector<uint8_t>);

    void plot(std::vector<float> data);

private:
    /**
     * @brief The path image.
     */
    cv::Mat path_;
    /**
     * @brief True if path image is already initialized, false if not.
     */
    bool is_path_initialized_;
    /**
     * @brief Current position in the path image.
     *
     * This variable holds the number of the column of the path image, to which the next line, given via paintPath(),
     * will be painted.
     */
    unsigned int path_pos_;
};

#endif // VISUALIZATION_H
