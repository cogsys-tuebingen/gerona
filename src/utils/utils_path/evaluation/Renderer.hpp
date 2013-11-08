/*
 * MapRenderer.hpp
 *
 *  Created on: Feb 8, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef MAPRENDERER_HPP
#define MAPRENDERER_HPP

/// COMPONENT
#include "Util.hpp"
#include "../generic/Common.hpp"

/// PROJECT
#include <utils_generic/Utils.hpp>
#include <utils_general/CDUniTuebingen.hpp>

/// SYSTEM
#include <boost/static_assert.hpp>
#include <boost/utility.hpp>
#include <boost/foreach.hpp>
#include <opencv2/opencv.hpp>

namespace lib_path
{

template < int SCALE, class SearchAlgorithm >
struct MapRenderer : public SearchAlgorithm {
    enum { Scale = SCALE };

    typedef typename SearchAlgorithm::NodeT NodeType;
    typedef typename SearchAlgorithm::PathT PathT;
    typedef typename SearchAlgorithm::NodeT NodeT;
    typedef typename SearchAlgorithm::PointT PointT;
    typedef typename SearchAlgorithm::MapT MapT;
    typedef typename SearchAlgorithm::Heuristic Heuristic;

    using SearchAlgorithm::map_;
    using SearchAlgorithm::start;
    using SearchAlgorithm::goal;

    MapRenderer()
        : has_focus(false), has_prev(false), max_dist(1)
    {}

    bool usesOrientation() const {
        return map_.theta_slots > 1;
    }

    int noExpansions() const {
        return SearchAlgorithm::expansions;
    }

    int noMultiExpansions() const {
        return SearchAlgorithm::multi_expansions;
    }

    int noUpdates() const {
        return SearchAlgorithm::updates;
    }

    virtual void setMap(const MapT* map) {
        SearchAlgorithm::setMap(map);

        img_w = map_.getMap()->getWidth() * Scale;
        img_h = map_.getMap()->getHeight() * Scale;

        map_w = map_.w;
        map_h = map_.h;

        map_data = map_.data;
    }

    void visualizeSearchSpace() {
        assert(goal != NULL);

        findFocus();

        analyze();
        visualizeSearchSpaceGrid();
        visualizeSearchSpaceGraph();
    }

    void findFocus() {
        if(has_focus) {
            return;
        }

        int wh = img_w / 2;
        int hh = img_h / 2;
        setFocus(wh, hh, std::max(wh, hh));
    }

#define FOREACH_NODE(name) { \
    unsigned int i = 0; \
    for(unsigned t = 0; t < map_.theta_slots; ++t) {\
    for(unsigned y = 0; y < map_h; ++y) {\
    for(unsigned x = 0; x < map_w; ++x) {\
    name = map_data[i++]; \
    if(node.distance == INFINITY) {continue;}\
    if(node.distance == std::numeric_limits<double>::max()) {continue;}
#define END_FOREACH_NODE }}}}

    void analyze() {
        max_dist = 1.0;
        has_prev = false;
        FOREACH_NODE(const NodeType& node) {
            if(node.distance > max_dist) {
                max_dist = node.distance;
            }
            if(node.prev != NULL) {
                has_prev = true;
            }
            END_FOREACH_NODE }
    }

    void visualizeSearchSpaceGrid() {
        int w = Scale-1;

        static cv::Scalar stages[] = {
            //            uni_tuebingen::cd::secondary::light_green,
            cv::Scalar::all(255),
            uni_tuebingen::cd::secondary::green,
            //            uni_tuebingen::cd::secondary::dark_green,
            //            uni_tuebingen::cd::secondary::dark_blue,
            //            uni_tuebingen::cd::secondary::blue,
            //            uni_tuebingen::cd::secondary::cyan,
            uni_tuebingen::cd::secondary::orange,
            uni_tuebingen::cd::secondary::red,
            //            uni_tuebingen::cd::secondary::brown,
            uni_tuebingen::cd::primary::anthrazite
        };
        int stage_count = sizeof(stages) / sizeof(cv::Scalar);

        FOREACH_NODE(const NodeType& node) {
            unsigned x_ = x * Scale;
            unsigned y_ = (map_h - y - 1) * Scale;

            double col_angle = node.distance / max_dist - 0.0001;

            double idx = col_angle * (stage_count-1);

            int a = std::floor(idx);
            int b = std::min(stage_count-1, a + 1);

            double fa = std::abs(idx - b);
            double fb = std::abs(idx - a);

            cv::Scalar col = stages[a] * fa + stages[b] * fb;

            //            cv::Scalar col = uni_tuebingen::cd::primary::anthrazite * (1-col_angle) +
            //                    uni_tuebingen::cd::secondary::cyan * col_angle;

            cv::rectangle(out_, cv::Point(x_, y_), cv::Point(x_+w,y_+w), col, CV_FILLED);

            END_FOREACH_NODE }
    }

    void visualizeSearchSpaceGraph() {
        //        if(has_prev){
        //            FOREACH_NODE(const NodeType& node) {
        //                if(node.prev) {
        //                    NodeType* prev = dynamic_cast<NodeType*>(node.prev);
        //                    cv::line(out_, p2cvRef(node, map_h, Scale), p2cvRef(*prev, map_h, Scale), cv::Scalar::all(255), Scale/2);
        //                    //                            cv::LineIterator it(out_, p2cvRef(node, map_h, Scale), p2cvRef(*prev, map_h, Scale));
        //                    //                            for(int i = 0; i < it.count; ++i, ++it) {
        //                    //                                cv::Vec3b& v = *(cv::Vec3b*) *it;
        //                    //                                v = cv::Vec3b(std::min(255, v[0]+d), std::min(255, v[1]+d), std::min(255, v[2]+d));
        //                    //                            }
        //                }
        //                END_FOREACH_NODE }
        //        }


        FOREACH_NODE(NodeType& node) {
            if(node.isMarked(NodeType::MARK_WATCHED)) {
                NodeType* pt = &node;
                cv::Scalar col = uni_tuebingen::cd::secondary::beige;
                if(usesOrientation()) {
                    NodeType* tmp = &node;
                    while(tmp->prev != NULL) {
                        NodeType* prev = dynamic_cast<NodeType*>(tmp->prev);
                        cv::line(out_, p2cvRef(*tmp, map_h, Scale), p2cvRef(*prev, map_h, Scale), uni_tuebingen::cd::primary::anthrazite, 1.5*Scale/2, CV_AA);
                        tmp = prev;
                    }
                    tmp = &node;
                    while(tmp->prev != NULL) {
                        NodeType* prev = dynamic_cast<NodeType*>(tmp->prev);
                        cv::line(out_, p2cvRef(*tmp, map_h, Scale), p2cvRef(*prev, map_h, Scale), colorFor(*tmp), Scale/2, CV_AA);
                        tmp = prev;
                    }
                }

                cv::Point p = p2cvRef(node, map_h, Scale);
                if(p.x >= 0 && p.x < out_.cols && p.y >= 0 && p.y < out_.rows) {
                    if(map_.theta_slots > 1) {
                        renderArrow(node, uni_tuebingen::cd::secondary::cyan, 0.5f);
                    } else {
                        float scale = 0.25f;
                        cv::circle(out_, p, 2 * Scale * scale, uni_tuebingen::cd::secondary::dark_blue, CV_FILLED, CV_AA);
                        cv::circle(out_, p, 1.5 * Scale * scale, colorFor(node), CV_FILLED, CV_AA);
                    }
                }
                node.unMark(NodeType::MARK_WATCHED);
            }
            END_FOREACH_NODE }


        if(!has_prev){
            FOREACH_NODE(NodeType& node) {
                if(node.isMarked(NodeType::MARK_EXPANDED)) {
                    cv::Point p = p2cvRef(node, map_h, Scale);
                    if(p.x >= 0 && p.x < out_.cols && p.y >= 0 && p.y < out_.rows) {
                        float scale = map_.theta_slots > 1 ? 1.0f : 0.25f;
                        cv::circle(out_, p, 3 * Scale * scale, uni_tuebingen::cd::secondary::dark_green, CV_FILLED, CV_AA);
                        cv::circle(out_, p, 2 * Scale * scale, uni_tuebingen::cd::secondary::green, CV_FILLED, CV_AA);
                    }
                    node.unMark(NodeType::MARK_EXPANDED);
                }
            }
            END_FOREACH_NODE }
    }

    inline void fill(generic::Int2Type<true>, unsigned x, unsigned y, uint8_t v) {
        out_.at<cv::Vec3b>(out_.rows-1-y, x) = cv::Vec3b(255, v, v);
    }

    template <int N>
    inline void fill(generic::Int2Type<N>, unsigned x, unsigned y, uint8_t v) {
        static const int W = N;

        unsigned sx = N * x;
        unsigned sy = N * y;

        for(unsigned dy = sy; dy < sy+W; ++dy) {
            for(unsigned dx = sx; dx < sx+W; ++dx) {
                out_.at<cv::Vec3b>(out_.rows-1-dy, dx) = cv::Vec3b(255, v, v);
            }
        }
    }

    void render_factor_mult(double d) {
        render_factor = std::max(0.2, render_factor * d);
        std::cout << "new factor:" << render_factor << std::endl;
    }

    void render_offset_add(double offset) {
        offset_ += offset;
    }

    void renderMap() {
        assert(out_.rows > 0);
        assert(out_.cols > 0);

        assert(goal != NULL);
        assert(start != NULL);

        findFocus();

        for(unsigned y=0; y<map_h; y++) {
            for(unsigned x=0; x<map_w; x++) {
                cv::Vec3b col;
                if(map_.isFree(x, y)) {
                    col = cv::Vec3b::all(255);

                    NodeT c;
                    c.x = x;
                    c.y = y;
                    c.theta = MathHelper::AngleClamp(goal->theta + offset_);

                    double heuristics = readHeuristics(c);
                    if(heuristics < 0) {
                        col = cv::Vec3b(220,200,200);
                    } else {
                        unsigned min = 100;
                        double span = 255.0-min;
                        unsigned v = 255 - std::min(span, std::max(0.0, ((heuristics + render_offset) * render_factor / 255.0 * span)));
                        col = cv::Vec3b(v, v, v);
                    }


                } else {
                    const cv::Scalar& s = uni_tuebingen::cd::primary::anthrazite;
                    col = cv::Vec3b(s[0], s[1], s[2]);
                }

                fill(generic::Int2Type<Scale>(), x, y, col);
            }
        }

        unsigned meterInCells = Scale / map_.getResolution();

        if(meterInCells > 2) {
            for(int y=0; y<out_.rows; y+=meterInCells) {
                for(int x=0; x<out_.cols; x+=meterInCells) {
                    cv::rectangle(out_, cv::Rect(x,y,meterInCells+1,meterInCells+1), uni_tuebingen::cd::primary::anthrazite, 2);
                }
            }
        }
    }

    void fill(generic::Int2Type<true>, unsigned x, unsigned y, cv::Vec3b& col) {
        out_.at<cv::Vec3b>(out_.rows-1-y, x) = col;
    }

    template <int N>
    void fill(generic::Int2Type<N>, unsigned x, unsigned y, cv::Vec3b& col) {
        static const int W = N;

        unsigned sx = N * x;
        unsigned sy = N * y;

        int i = 0;
        for(unsigned dy = sy; dy < sy+W; ++dy) {
            for(unsigned dx = sx; dx < sx+W; ++dx) {
                int yy = out_.rows-1-dy;
                int xx = dx;

                if(yy < 0 || xx < 0 || yy >= img_h || xx >= img_w) {
                    continue;
                }
                out_.at<cv::Vec3b>(yy, xx) = col;
            }
        }


        if(N > 2) {
            unsigned yy = out_.rows-1-sy-W;
            cv::rectangle(out_, cv::Rect(sx, yy, W+1, W+1), uni_tuebingen::cd::primary::anthrazite, 1);
        }
    }

    void renderPath(const PathT& path) {
        if(path.empty()) {
            return;
        }

        tracePath(path, uni_tuebingen::cd::primary::anthrazite, 1.25*Scale);
        tracePath(path, cv::Scalar::all(-1), 0.75*Scale);
    }

    void tracePath(const PathT& path, const cv::Scalar& color, double scale) {
        const NodeT* last = NULL;

        bool generate_color = color == cv::Scalar::all(-1);

        BOOST_FOREACH(const NodeT& pt, path) {
            if(last != NULL) {
                cv::Scalar c = (generate_color) ? colorFor(pt) : color;
                cv::line(out_, p2cvRef(*last, map_h, Scale), p2cvRef(pt, map_h, Scale), c, scale, CV_AA);
            }
            last = &pt;
        }
    }

    template <class Pose>
    void renderArrow(const Pose& pose, cv::Scalar color = uni_tuebingen::cd::secondary::dark_blue, float scale = 1.0f, cv::Scalar border_color = cv::Scalar::all(-1)) {
        Point2d t(pose);
        Point2d right(12, 5);
        Point2d left(12, -5);
        Point2d dir(18, 0);


        unsigned meterInCells = 1 / map_.getResolution();
        double f = (1/18.0) * scale * meterInCells;

        right *= f;
        left *= f;
        dir *= f;

        Point2d tip = t + dir.rotate(pose.theta);
        cv::line(out_, p2cv(pose, map_h, Scale), p2cv(t + dir.rotate(pose.theta), map_h, Scale),
                 border_color, 2.0f * scale * Scale, CV_AA);
        cv::line(out_, p2cv(tip, map_h, Scale), p2cv(t + left.rotate(pose.theta), map_h, Scale),
                 border_color, 2.0f * scale * Scale, CV_AA);
        cv::line(out_, p2cv(tip, map_h, Scale), p2cv(t + right.rotate(pose.theta), map_h, Scale),
                 border_color, 2.0f * scale * Scale, CV_AA);
        cv::line(out_, p2cv(pose, map_h, Scale), p2cv(t + dir.rotate(pose.theta), map_h, Scale),
                 color, 1.5f * scale * Scale, CV_AA);
        cv::line(out_, p2cv(tip, map_h, Scale), p2cv(t + left.rotate(pose.theta), map_h, Scale),
                 color, 1.5f * scale * Scale, CV_AA);
        cv::line(out_, p2cv(tip, map_h, Scale), p2cv(t + right.rotate(pose.theta), map_h, Scale),
                 color, 1.5f * scale * Scale, CV_AA);
    }

    template <class Pose>
    void fillCell(const Pose& pose, cv::Scalar color = uni_tuebingen::cd::secondary::dark_blue, cv::Scalar border_color = cv::Scalar::all(-1)) {
        cv::Point tl = p2cv(pose + Pose(0,1,0), map_h, Scale);
        cv::Point size(Scale, Scale);
        cv::rectangle(out_, tl, tl + size, color, CV_FILLED);

        if(border_color != cv::Scalar::all(-1)) {
            cv::rectangle(out_, tl, tl + size, border_color, std::ceil(Scale / 4.0));
        }
    }

    void setFocus(int x_img, int y_img, int radius) {
        int x = x_img / (double) Scale;
        int y = (out_.rows - y_img) / (double) Scale;

        double min = std::numeric_limits<double>::max();
        double max = std::numeric_limits<double>::min();

        for(int dy=-radius; dy<=radius; ++dy) {
            for(int dx=-radius; dx<=radius; ++dx) {
                int mx = x + dx;
                int my = y + dy;

                cv::Vec3b col;
                if(map_.getMap()->isInMap(mx, my) && map_.isFree(mx, my)) {
                    NodeT c;
                    c.x = mx;
                    c.y = my;
                    c.theta = goal->theta + offset_;

                    double v = readHeuristics(c);
                    if(v == std::numeric_limits<double>::max()) {
                        continue;
                    }

                    if(v > max) {
                        max = v;
                    }
                    if(v < min) {
                        min = v;
                    }
                }
            }
        }

        has_focus = true;

        render_offset = -min;
        render_factor = 255.0 / (max - min);
    }


    void setOut(cv::Mat img) {
        out_ = img;
    }

    cv::Mat getOut() {
        return out_;
    }


    // specializations
    template <class T>
    cv::Scalar colorFor(const T& pt, typename boost::enable_if_c<NodeTraits<T>::HasForwardField, T>::type* = 0) {
        return pt.forward ? uni_tuebingen::cd::secondary::cyan : uni_tuebingen::cd::secondary::lila;
    }
    template <class T>
    cv::Scalar colorFor(const T& pt, typename boost::disable_if_c<NodeTraits<T>::HasForwardField, T>::type* = 0) {
        return uni_tuebingen::cd::secondary::cyan;
    }

    template <class T>
    double readHeuristics(T& pt, typename boost::enable_if_c<NodeTraits<T>::HasHeuristicField, T>::type* = 0) {
        Heuristic::compute(&pt, goal, map_.getResolution());
        return pt.h;
    }
    template <class T>
    double readHeuristics(T& pt, typename boost::disable_if_c<NodeTraits<T>::HasHeuristicField, T>::type* = 0) {
        return -1;
    }


protected:
    cv::Mat out_;
    cv::Mat tree_;

    unsigned img_w;
    unsigned img_h;

    NodeType * map_data;
    unsigned map_w;
    unsigned map_h;

    double render_factor;
    double render_offset;
    double offset_;

    bool has_focus;
    bool has_prev;
    double max_dist;
};

}

#endif // MAPRENDERER_HPP
