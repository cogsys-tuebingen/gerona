/// HEADER
#include <path_follower/controller/vector_field_histogram.h>

/// SYSTEM
#include <opencv2/opencv.hpp>

#define ANGLE_RAD_TO_IDX(ang) (((ang) + M_PI) * (180.0 / M_PI)) / ang_res_
#define IDX_TO_ANGLE_RAD(idx) (((idx) * ang_res_) / (180.0 / M_PI)) - M_PI

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

    double fov_deg = 100.0 / 2.0;
    double fov_rad = (fov_deg / 180.0) * M_PI;

    histogram_.clear();
    histogram_.resize(n, 0);

    double ws = influence_distance /*m*/;

    debug_map_ = cv::Mat(map_.info.height, map_.info.width, CV_32FC3, cv::Scalar::all(127));

    for(unsigned int y = 0; y < map_.info.height; ++y) {
        for(unsigned int x = 0; x < map_.info.width; ++x) {
            double px = x * res;
            double py = y * res;

            double dy = py - ry;
            double dx = px - rx;
            double angle = std::atan2(dy, dx);

            if(angle < -fov_rad || angle > fov_rad) {
                continue;
            }

            double d = std::sqrt(dx*dx + dy*dy);
            if(d > ws) {
                continue;
            }


            int map_index = y * map_.info.width + x;
            double c = map_.data.at(map_index) / 100.0;

            if(c > 0.45 && c < 0.55) {
                continue;
            }

            // TODO: find values:
            double b = 0.15;
            double a = 1 + b * std::pow((ws - 1.0) / 2.0, 2);

            double magnitude = c*c * (a - b * d*d);

            int histogram_index = ANGLE_RAD_TO_IDX(angle);

            histogram_[histogram_index] += magnitude;

            debug_map_.at<cv::Vec3f>(y,x) = cv::Vec3f(magnitude,0.f,0.f);
        }
    }

    smooth();

    double max_v = *std::max_element(histogram_.begin(), histogram_.end());
    no_obstacle_ = max_v < threshold;

    // find valleys
    bool in_valley = false;
    int valley_id = -1;

    valley_.clear();
    valley_.resize(n, -1);

    valley_begin_.clear();
    valley_end_.clear();;
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

    if(in_valley) {
        // last valley is not closed
        valley_end_[valley_id] = n-1;
    }

    // check which valley is wide and which is narrow
    for(size_t i = 0; i <= valley_.size(); ++i) {
        int valley = valley_[i];
        is_valley_wide_[valley] = valley_width_[valley] > s_max_;
    }
}

void VectorFieldHistogram::smooth()
{
    std::vector<double> smoothed_histogram;

    int n = histogram_.size();
    static const int l = 3; // see VFH paper

    for(int k = 0; k < n; ++k) {
        double v = 0;
        for(int i = - l; i <= l; ++i) {
            // circular normalization
            int j = (k + i + n) % n;
            assert(j >= 0);
            assert(j < n);

            int f = (l - std::abs(i)) + 1;

            assert(i != -l || f == 1);
            assert(i != 0 || f == (l + 1));
            assert(i != l || f == 1);

            v += f * histogram_[j];
        }

        smoothed_histogram.push_back(v / (2 * l + 1));
    }

    histogram_ = smoothed_histogram;
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

    if(no_obstacle_) {
        result_course = course;
        return true;
    }


    int n = histogram_.size();
    int k_targ = ANGLE_RAD_TO_IDX(course);

    // find valley closest to k_targ
    int closest_valley = -1;
    int closest_dist = 2 * n;

    for(size_t i = 0; i < valley_.size(); ++i) {
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
        if(d_begin < d_end) {
            near_ = begin;
        } else {
            near_ = end;
        }

        if(near_ < k_targ) {
            far_ = near_ - s_max_;
        } else {
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

    kf_ = (near_ + far_) / 2;
    result_course = IDX_TO_ANGLE_RAD(kf_);
//    result_course = course;
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

    for(std::size_t i = 0; i < n; ++i) {
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
        cv::line(hist_img, cv::Point(begin_x, 0), cv::Point(begin_x, hist_img.rows), cv::Scalar(255,0,0), 3);
        cv::line(hist_img, cv::Point(end_x + w, 0), cv::Point(end_x + w, hist_img.rows), cv::Scalar(0,0,255), 3);
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
    cv::line(hist_img, cv::Point(center_x, h/4), cv::Point(center_x, hist_img.rows-h/4), cv::Scalar(0,0,255), 3);

    // draw course
    int course_x = w * ANGLE_RAD_TO_IDX(course);
    cv::line(hist_img, cv::Point(course_x, h/4), cv::Point(course_x, hist_img.rows-h/4), cv::Scalar(255,0,0), 3);

    // draw course
    int r_course_x = w * kf_;
    cv::line(hist_img, cv::Point(r_course_x, 0), cv::Point(r_course_x, hist_img.rows), cv::Scalar(0,200,0), 5);

    // draw threshold
    int t = threshold * normalizer;
    cv::line(hist_img, cv::Point(0, t), cv::Point(hist_img.cols, t), cv::Scalar(0,0,255), 1);

    cv::flip(hist_img, hist_img, -1);

//    cv::imshow("map", debug_map_);
    cv::imshow("histogram", hist_img);
    cv::waitKey(20);
}
