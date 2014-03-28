/// HEADER
#include "vector_field_histogram.h"

/// SYSTEM
#include <opencv2/opencv.hpp>

#define ANGLE_RAD_TO_IDX(ang) (((ang) + M_PI) * (180.0 / M_PI)) / ang_res_;

VectorFieldHistogram::VectorFieldHistogram()
    : has_map_(false), selected_valley_(-1)
{
    ang_res_ = 5 /*°*/;
    s_max_ = 18;
}

void VectorFieldHistogram::setMap(const nav_msgs::OccupancyGrid &map)
{
    map_ = map;
    has_map_ = true;
}

void VectorFieldHistogram::create(double influence_distance, double threshold)
{
    if(!has_map_) {
        return;
    }

    double res = map_.info.resolution;
    double rx = -map_.info.origin.position.x;
    double ry = -map_.info.origin.position.y;

    assert((360 % ang_res_) == 0);
    int n = 360 / ang_res_;


    histogram_.clear();
    histogram_.resize(n, 0);

    double ws = influence_distance /*m*/;

    debug_map_ = cv::Mat(map_.info.height, map_.info.width, CV_32FC3, cv::Scalar::all(127));

    for(int y = 0; y < map_.info.height; ++y) {
        for(int x = 0; x < map_.info.width; ++x) {
            double px = x * res;
            double py = y * res;

            double dy = py - ry;
            double dx = px - rx;

            double d = std::sqrt(dx*dx + dy*dy);
            if(d > ws) {
                continue;
            }


            int map_index = y * map_.info.width + x;
            double c = map_.data.at(map_index) / 100.0;


            // TODO: find values:
            double b = 0.15;
            double a = 1 + b * std::pow((ws - 1.0) / 2.0, 2);

            double angle = std::atan2(dy, dx);
            double magnitude = c*c * (a - b * d*d);

            int histogram_index = ANGLE_RAD_TO_IDX(angle);

            histogram_[histogram_index] += magnitude;

            debug_map_.at<cv::Vec3f>(y,x) = cv::Vec3f(magnitude,0.f,0.f);
        }
    }

    // find valleys
    bool in_valley = false;
    int valley_id = -1;

    valley_.clear();
    valley_.resize(n, -1);

    valley_width_.clear();
    is_valley_wide_.clear();

    selected_valley_ = -1;

    // cluster bins
    for(int i = 0; i < n; ++i) {
        double val = histogram_.at(i);
        bool valley = (val < threshold);
        if(valley) {
            if(!in_valley) {
                in_valley = true;
                ++valley_id;

                valley_begin_[valley_id] = i;
                valley_width_[valley_id] = 0;
            }
            valley_[i] = valley_id;
            ++valley_width_[valley_id];

        } else if(in_valley) {
            in_valley = false;
            valley_end_[valley_id] = i - 1;
        }
    }

    // check which valley is wide and which is narrow
    for(int i = 0; i <= valley_.size(); ++i) {
        int valley = valley_[i];
        is_valley_wide_[valley] = valley_width_[valley] > s_max_;
    }
}

namespace {
int valley_dist(int v, int k_targ, int n) {
    return std::abs(v - k_targ) % n;
}
}
bool VectorFieldHistogram::adjust(double course, double threshold, double& result_course)
{
    if(!has_map_) {
        result_course = course;
        return true;
    }

    if(valley_.empty()) {
        result_course = course;
        return false;
    }


    int n = histogram_.size();
    int k_targ = ANGLE_RAD_TO_IDX(course);

    // find valley closest to k_targ
    int closest_valley = -1;
    int closest_dist = 2 * n;

    for(int i = 0; i < valley_.size(); ++i) {
        int valley = valley_[i];
        int begin = valley_begin_[valley];
        int end = valley_end_[valley];

        if(valley_dist(begin, k_targ, n) < closest_dist) {
            closest_dist = valley_dist(begin, k_targ, n);
            closest_valley = valley;
        }
        if(valley_dist(end, k_targ, n) < closest_dist) {
            closest_dist = valley_dist(end, k_targ, n);
            closest_valley = valley;
        }

    }

    // select closest valley
    selected_valley_ = closest_valley;

    // calculate borders
    int begin = valley_begin_[selected_valley_];
    int end = valley_end_[selected_valley_];

    int d_begin = valley_dist(begin, k_targ, n);
    int d_end = valley_dist(end, k_targ, n);

    if(is_valley_wide_[selected_valley_]) {
        /// WENN VIEL PLATZ LINKS UND RECHTS, WARUM NICHT ZENTRIERT?!!?!?!?!?!
        if(d_begin < d_end) {
            near_ = begin;
            far_ = near_ + s_max_;
        } else {
            near_ = end;
            far_ = near_ + s_max_;
        }
    } else {
        if(d_begin < d_end) {
            near_ = begin;
            far_ = end;
        } else {
            near_ = end;
            far_ = begin;
        }
    }

    result_course = course;
    return true;
}

bool VectorFieldHistogram::isReady()
{
    return has_map_;
}

void VectorFieldHistogram::visualize(double course, double threshold)
{
    // draw debug polar image
    int w = 8;
    int h = 200;
    double max_v = *std::max_element(histogram_.begin(), histogram_.end());
    double normalizer =  (h / max_v);

    std::size_t n = histogram_.size();
    cv::Mat hist_img(h,w*n, CV_8UC3, cv::Scalar::all(0));

    for(int i = 0; i < n; ++i) {
        double val = histogram_.at(i);
        int col_h = normalizer * val;
        // background
        int valley = valley_[i];
        if(valley >= 0) {
            cv::Rect rec(i * w, 0, w, h);
            cv::Scalar color(50,is_valley_wide_[valley] ? 200 : 100,50);

            cv::rectangle(hist_img, rec, color, CV_FILLED);
        }

        // bar
        cv::Rect rec(i * w, 0, w, col_h);
        cv::Scalar color(255,255,255);
        cv::rectangle(hist_img, rec, color, CV_FILLED);
    }

    // draw valley borders
    for(std::size_t i = 0; i < valley_.size(); ++i) {
        int valley_id = valley_[i];
        int begin_x = valley_begin_[valley_id] * w;
        int end_x = valley_end_[valley_id] * w;
        cv::line(hist_img, cv::Point(begin_x, 0), cv::Point(begin_x, hist_img.rows), cv::Scalar(255,255,0), 1);
        cv::line(hist_img, cv::Point(end_x + w, 0), cv::Point(end_x + w, hist_img.rows), cv::Scalar(0,255,255), 1);
    }

    // highlight selected valley
    if(selected_valley_ != -1) {
        int begin_x = valley_begin_[selected_valley_] * w;
        int end_x = valley_end_[selected_valley_] * w;

        cv::Rect rec(begin_x, 2, std::abs(begin_x - end_x) + w, h-4);
        cv::Scalar color(0, 255, 0);
        cv::rectangle(hist_img, rec, color, 2);

        int near_x = near_ * w;
        int far_x = far_ * w;
        cv::line(hist_img, cv::Point(near_x, h/4), cv::Point(near_x, hist_img.rows-h/4), cv::Scalar(127,127,255), 4);
        cv::line(hist_img, cv::Point(far_x, h/4), cv::Point(far_x, hist_img.rows-h/4), cv::Scalar(255,127,127), 4);
    }

    // draw center line (forward looking)
    int center_x = hist_img.cols / 2;
    cv::line(hist_img, cv::Point(center_x, 0), cv::Point(center_x, hist_img.rows), cv::Scalar(0,0,255), 2);

    // draw course
    int course_x = w * ANGLE_RAD_TO_IDX(course);
    cv::line(hist_img, cv::Point(course_x, 0), cv::Point(course_x, hist_img.rows), cv::Scalar(255,0,0), 2);

    // draw threshold
    int t = threshold * normalizer;
    cv::line(hist_img, cv::Point(0, t), cv::Point(hist_img.cols, t), cv::Scalar(0,0,255), 1);

    cv::flip(hist_img, hist_img, -1);

    cv::imshow("map", debug_map_);
    cv::imshow("histogram", hist_img);
    cv::waitKey(30);
}
