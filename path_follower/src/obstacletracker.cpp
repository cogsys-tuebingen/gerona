#include "obstacletracker.h"

#include <boost/foreach.hpp>

using namespace std;

void ObstacleTracker::update(std::vector<cv::Point2f> observed_obstacles)
{
    /**
     * There is a slight difference in the actual implementation of the algorithmen, compared to the description in the
     * class comment:
     * Since removing rows/columns from the distance matrix changes the indices of the remaining entries, it would be
     * quite complicated to keep the connetion between the obstacles and the entries in the matrix. Therefore, instead
     * of cutting the distance matrix ('dist'), an additional matrix 'd_idx' is constructed, containing indices to the
     * fields in the distance matrix. This index-matrix is then used to cut away matched obstacles, instead of the
     * distance matrix itself.
     */

    // construct distanc matrix. D[i,j] = distance between observed obstacle i and tracked obstacle j.
    cv::Mat dist(observed_obstacles.size(), obstacles_.size(), CV_32F);
    cv::Mat d_idx(dist.rows, dist.cols, CV_16U);
    vector<bool> obs_matched(observed_obstacles.size(), false);

    ushort i = 0;
    for (size_t o = 0; o < observed_obstacles.size(); ++o)
    for (size_t t = 0; t < obstacles_.size();         ++t) {
        dist.at<float>(o, t)   = norm(observed_obstacles[o] - obstacles_[t].last_position);
        d_idx.at<ushort>(o, t) = i++; // ATTENTION: This is the data-index to dist. It will only work, if this nested
                                      //            loop iterates over rows first!
    }

    //ROS_DEBUG_STREAM("D =\n" << dist);

    // now match the points
    vector<TrackedObstacle> matched_tracked;

    while (!d_idx.empty()) {
        //ROS_DEBUG_STREAM("Didx =\n" << d_idx);

        // min_idx = argmin(D)
        float min = dist.at<float>(d_idx.at<ushort>(0));
        int min_idx = 0;
        for (int i = 1; i < d_idx.rows*d_idx.cols; ++i) {
            int idx = d_idx.at<ushort>(i);
            if (dist.at<float>(idx) < min) {
                min = dist.at<float>(idx);
                min_idx = i;
            }
        }

        //ROS_DEBUG("min idx = %d (%d), dist = %g", d_idx.at<ushort>(min_idx), min_idx, dist.at<float>(d_idx.at<ushort>(min_idx)));

        // If the shortest distance is to big, assume that all matchable obstacles are matched.
        if (min > max_dist_) {
            break;
        }

        // data-index to distance-matrix coords (mi, mj are the indices of the matched obstacles)
        int mi = d_idx.at<ushort>(min_idx) / dist.cols;
        int mj = d_idx.at<ushort>(min_idx) % dist.cols;
        // ... and to index-matrix coords (those are only needed, to remove the corresponding row and column, a few lines below)
        int mi_idx = min_idx / d_idx.cols;
        int mj_idx = min_idx % d_idx.cols;

        //ROS_DEBUG("matched %d, %d (%d,%d)", mi, mj, mi_idx, mj_idx);

        // match observed obstacle mi with tracked obstacle mj
        obstacles_[mj].last_position = observed_obstacles[mi];
        matched_tracked.push_back(obstacles_[mj]);
        // mark mi as matched.
        obs_matched[mi] = true;

        // delete mi and mj from the distances index matrix
        removeRow(d_idx, mi_idx);
        if (d_idx.empty())
            break; // stop before calling removeColumn on an empty matrix will cause a crash.
        removeColumn(d_idx, mj_idx);
    }

    // Delete tracked obstacles, which could not be matched (assume that they left the path). This is done by
    // overwriting the old list of tracked obstacles by the list of matched tracked obstacles.
    obstacles_ = matched_tracked;

    // If there are unmatched obstacles in the observation, add them as new obstacles
    for (size_t i = 0; i < observed_obstacles.size(); ++i) {
        if (!obs_matched[i]) {
            TrackedObstacle to;
            to.last_position = observed_obstacles[i];
            to.time_of_first_sight = ros::Time::now();

            obstacles_.push_back(to);
        }
    }
}

void ObstacleTracker::removeRow(cv::Mat &m, int row)
{
    assert(row >= 0 && row < m.rows);

    // {v|h}concat can not handle empty matrices, thus there are some nasty case distinctions necessary...
    // Beware: end row/col of range-methods is exclusive!
    if (row == 0) {
        m = m.rowRange(1,m.rows);
    } else if (row == m.rows-1) {
        m = m.rowRange(0,row);
    } else {
        cv::vconcat(m.rowRange(0,row), m.rowRange(row+1, m.rows),  m);
    }
}

void ObstacleTracker::removeColumn(cv::Mat &m, int col)
{
    assert(col >= 0 && col < m.cols);

    // {v|h}concat can not handle empty matrices, thus there are some nasty case distinctions necessary...
    // Beware: end col/col of range-methods is exclusive!
    if (col == 0) {
        m = m.colRange(1,m.cols);
    } else if (col == m.cols-1) {
        m = m.colRange(0,col);
    } else {
        cv::hconcat(m.colRange(0,col), m.colRange(col+1, m.cols),  m);
    }
}
