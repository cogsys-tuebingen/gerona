/*
 * EMMaxTest.cpp
 *
 *  Created on: Feb 04, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

/// COMPONENT
#include "Clustering.h"

/// PROJECT
#include <utils/LibUtil/Stopwatch.h>

/// SYSTEM
#include <boost/bind.hpp>
#include <cmath>
#include <opencv2/opencv.hpp>

using namespace lib_clustering;

typedef std::pair<cv::Point, uchar> PointT;
typedef std::vector<PointT> PointList;

typedef KMeans<2, RandomInitialization, EuclideanDistance, Dense, uchar* > EMMax;

// register the custom point type for the generic k means implementation
namespace lib_clustering
{
template <>
struct AccessTraits<int, PointList> {
    static int index(const PointT& in, unsigned index) {
        switch(index) {
        default:
        case 0:
            return in.first.x;
        case 1:
            return in.first.y;
        }
    }
    static int value(const PointT& in) {
        return in.second;
    }
    template <class VectorT, class IndexProvider, class Type>
    static VectorT readVector(const PointT& in, Type& val) {
        VectorT vector(val);
        copy(vector, in);
        return vector;
    }
    template <class VectorT, typename IndexProvider>
    static VectorT readEmptyVector(const IndexProvider& in) {
        VectorT vector;
        copy(vector, in);
        return vector;
    }

    template <class VectorT, typename IndexProvider>
    static void copy(VectorT& vector, const IndexProvider& in) {
        for(unsigned dim = 0; dim < VectorT::Dimension; ++dim) {
            vector[dim] = AccessTraits::index(in, dim);
        }
    }
};
}

/**
 * @brief The EMMaxTest class wraps a test scenario
 */
class EMMaxTest
{
public:
    EMMaxTest(int k, long seed)
        : k(k), seed(seed), algo(k) {
    }

    void loop() {
        double avgs[] = {0, 0};
        std::string experiments[] = { "EM    ", "opencv"};
        int experiment_counter = 0;
        bool draw_results = true;

        do {
            srand(seed);

            makeRandomSet();

            int experiment = 0;

            {
                window = experiments[experiment];
                Stopwatch stop;
                algo.find(dataimage.data, limits, centers, /*boost::bind(&EMMaxTest::draw, this),*/ seed);
                avgs[experiment] += stop.usElapsed();
                experiment++;
                if(draw_results) draw();
            }

            {
                // CV version
                window = experiments[experiment];
                cv::Mat data(datapoints.size(), 2, CV_32FC1);
                int row = 0;
                for(PointList::iterator it = datapoints.begin(); it != datapoints.end(); ++it) {
                    data.at<float>(row, 0) = it->first.x;
                    data.at<float>(row, 1) = it->first.y;

                    row ++;
                }

                cv::Mat labels, centers;
                Stopwatch stop;
                cv::EM em(k);
                em.train(data, cv::noArray(), labels);
                avgs[experiment] += stop.usElapsed();
                experiment++;
                if(draw_results) {
                    cv::Mat centers = em.get<cv::Mat>("means");
                    drawCV(centers, data, labels);
                }
            }

            experiment_counter++;
            seed++;



            double max = 1;
            int bars = 20;
            for(int e = 0; e < experiment; ++e) {
                max = std::max(max, avgs[e] / experiment_counter);
            }
            for(int e = 0; e < experiment; ++e) {
                double avg = avgs[e] / experiment_counter;
                std::cout << experiments[e] << ":\t" << avg * 1e-3 << "ms \t [ ";
                int top = bars * avg / max;
                for(int b = 0; b <= top; ++b) {
                    std::cout << "##";
                }
                for(int b = top; b < bars; ++b) {
                    std::cout << "  ";
                }
                std::cout << " ]\n";
            }

            std::cout << std::flush;

        } while(!draw_results || ((cv::waitKey(0) & 0xFF) != 27 && cvGetWindowHandle(window.c_str())));
    }


    bool inside(int v, int maximum) {
        return 0 <= v && v < maximum;
    }

    cv::Scalar color(int i, int k) {
        cv::Mat col(1,1,CV_8UC3, cv::Scalar((255.0 * i) / k, 255, 255));
        cv::cvtColor(col, col, CV_HSV2BGR);
        cv::Vec3b s = col.at<cv::Vec3b>(0,0);
        return cv::Scalar(s[0], s[1], s[2]);
    }

    void draw() {
        cv::Mat result(h, w, CV_8UC3);
        cv::cvtColor(dataimage, result, CV_GRAY2BGR);

        for(unsigned i = 0; i < centers.size(); ++i) {
            EMMax::ClusterT& c = centers[i];

            cv::Scalar col = color(i, k);

            for(unsigned j = 0; j < centers[i].members.size(); ++j) {
                //cv::circle(result, cv::Point((*centers[i].members[j])[0], (*centers[i].members[j])[1]), 5, col, 1, CV_AA);
                cv::circle(result, cv::Point((*centers[i].members[j])[0], (*centers[i].members[j])[1]), 2, col, CV_FILLED, CV_AA);
                //result.at<cv::Vec3b>((*centers[i].members[j])[1], (*centers[i].members[j])[0]) = cv::Vec3b(col[0], col[1], col[2]);
            }

            cv::circle(result, cv::Point(c.centroid[0], c.centroid[1]), 5, col, 1, CV_AA);
        }

        cv::imshow(window.c_str(), result);

        if((cv::waitKey(100) & 0xFF) == 27 || !cvGetWindowHandle(window.c_str())) {
            exit(0);
        }
    }


    void drawCV(cv::Mat centers, cv::Mat data, cv::Mat out) {
        cv::Mat result(h, w, CV_8UC3);
        cv::cvtColor(dataimage, result, CV_GRAY2BGR);

        for(unsigned i = 0; i < centers.rows; ++i) {

            cv::Scalar col = color(i, k);

            for(unsigned j = 0; j < out.rows; ++j) {
                if(out.at<int>(j) == i) {
                    cv::Mat sample = data.row(j);
                    cv::circle(result, cv::Point(sample.at<float>(0,0), sample.at<float>(0,1)), 2, col, CV_FILLED, CV_AA);
                }
            }

            cv::Mat c = centers.row(i);
            assert(c.type() == CV_64FC1);
            cv::circle(result, cv::Point(c.at<double>(0,0), c.at<double>(0,1)), 5, col, 1, CV_AA);
        }

        cv::imshow(window.c_str(), result);

        if((cv::waitKey(100) & 0xFF) == 27 || !cvGetWindowHandle(window.c_str())) {
            exit(0);
        }
    }

    void makeRandomSet() {
        dataimage = cv::Mat(h, w, CV_8UC1, cv::Scalar(0));
        datapoints.clear();

        int rand_clusters = std::max(k/2, rand() % int(k * 2));


        std::cout << "creating " << rand_clusters << " clusters" << std::endl;

        for(int cluster = 0; cluster < rand_clusters; ++cluster) {
            int spread = std::max(10, rand() % (std::min(w, h) / 4));
            int rand_pts = std::max(5, rand() % 300);
            int cx = std::abs(rand()) % w;
            int cy = std::abs(rand()) % h;

            for(int pt = 0; pt < rand_pts; ++pt) {
                double r = (rand() % spread);
                double angle = (rand() % 360) / M_PI * 180.0;
                int x = cx + r * std::cos(angle);
                int y = cy + r * std::sin(angle);

                uchar val = (rand() % 5);

                if(inside(y, h) && inside(x, w)) {
                    dataimage.at<uchar>(y, x) += val * 5;

                    if(dataimage.at<uchar>(y, x) == 255) {
                        std::cout << "warning, saturated" << std::endl;
                    }

                    for(unsigned i = 0; i < val; ++i) {
                        datapoints.push_back(PointT(cv::Point(x, y), 5));
                    }
                }
            }
        }


        limits.clear();
        limits.push_back(EMMax::LimitPair(0, w));
        limits.push_back(EMMax::LimitPair(0, h));
    }


private:
    static const int w = 800;
    static const int h = 600;

    int k;

    long seed;

    EMMax algo;

    cv::Mat dataimage;
    PointList datapoints;

    std::string window;

    std::vector<EMMax::ClusterT> centers;

    EMMax::LimitPairList limits;
};

int main(int argc, char** argv)
{
    int k = 8;
    if(argc > 1) {
        k = atoi(argv[1]);
        if(k <= 0 || k > 1024) {
            std::cout << "k=" << k << "is invalid" << std::endl;
            exit(2);
        }
    }
    EMMaxTest test(k, 1337);
    test.loop();
}
