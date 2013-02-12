/*
 * CostPrecomputation.cpp
 *
 *  Created on: Feb 12, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

/// PROJECT
#include "../common/SimpleGridMap2d.h"
#include "CurveGenerator.h"
#include "CurveRenderer.h"

/// SYSTEM
#include <boost/exception/all.hpp>
#include <boost/program_options.hpp>
#include <fstream>
#include <opencv2/opencv.hpp>

namespace po = boost::program_options;
using namespace lib_path;

void precompute(const std::string& dir, int dimension, int angles)
{

    lib_path::CurveGenerator generator;

    //////// CSC
    generator.parse("LSL");
    generator.parse("LSR");
    generator.parse("RSL");
    generator.parse("RSR");
    generator.parse("|LSL");
    generator.parse("|LSR");
    generator.parse("|RSL");
    generator.parse("|RSR");
    generator.parse("|L|SL");
    generator.parse("|L|SR");
    generator.parse("|R|SL");
    generator.parse("|R|SR");
    generator.parse("LS|L");
    generator.parse("LS|R");
    generator.parse("RS|L");
    generator.parse("RS|R");
    generator.parse("L|SL");
    generator.parse("L|SR");
    generator.parse("R|SL");
    generator.parse("R|SR");
    generator.parse("L|S|L");
    generator.parse("L|S|R");
    generator.parse("R|S|L");
    generator.parse("R|S|R");
    generator.parse("|LS|L");
    generator.parse("|LS|R");
    generator.parse("|RS|L");
    generator.parse("|RS|R");
    generator.parse("|L|S|L");
    generator.parse("|L|S|R");
    generator.parse("|R|S|L");
    generator.parse("|R|S|R");

    ///// CCC
    generator.parse("LRL");
    generator.parse("RLR");
    generator.parse("L|R|L");
    generator.parse("R|L|R");
    generator.parse("|L|R|L");
    generator.parse("|R|L|R");
    generator.parse("LR|L");
    generator.parse("RL|R");
    generator.parse("|LR|L");
    generator.parse("|RL|R");
    generator.parse("L|RL");
    generator.parse("R|LR");
    generator.parse("|L|RL");
    generator.parse("|R|LR");
    generator.parse("LR(b)|L(b)R");
    generator.parse("|LR(b)|L(b)R");
    generator.parse("RL(b)|R(b)L");
    generator.parse("|RL(b)|R(b)L");
    generator.parse("L|R(b)L(b)|R");
    generator.parse("R|L(b)R(b)|L");
    generator.parse("|L|R(b)L(b)|R");
    generator.parse("|R|L(b)R(b)|L");

    double curve_rad = 1.5; // m
    double res = 0.1f;

    double anglestep = 2 * M_PI / angles;

    Pose2d target(dimension / 2., dimension / 2., 0);

    SimpleGridMap2d map_info(dimension, dimension, res);
    map_info.setOrigin(Point2d(0, 0));
    map_info.setLowerThreshold(20);
    map_info.setUpperThreshold(50);

    generator.set_circle_radius(curve_rad);
    generator.set_cost_backwards(3.0);

    std::vector<std::pair<double, std::vector<double> > > costs;

    for(int angle = 0; angle < angles; ++angle) {
        double angle_rad = angle * anglestep;

        double min_cost = NOT_FREE;
        double max_cost = 0;

        std::vector<double> data(dimension*dimension);

        cv::Mat debug(dimension, dimension, CV_8UC3, cv::Scalar::all(128));

        int i = 0;
        for(int y = 0; y < dimension; ++y) {
            for(int x = 0; x < dimension; ++x) {
                Pose2d start(x,y, angle_rad);
                Curve* c = generator.find_path(start, target, &map_info, true);

                double v = c->weight();

                delete c;

                if(v < NOT_FREE) {
                    data[i++] = v;

                    max_cost = std::max(v, max_cost);
                    min_cost = std::min(v, min_cost);
                } else {
                    data[i++] = 0;
                }
            }

            for(int yy = 0; yy < dimension; ++yy) {
                unsigned ws = yy * dimension;
                for(int xx = 0; xx < dimension; ++xx) {
                    debug.at<cv::Vec3b>(yy,xx) = cv::Vec3b::all(255 * (data[ws + xx] - min_cost) / (max_cost - min_cost));
                }
            }

            std::stringstream ss;
            cv::Mat output;
            ss << "computing (angle=" << angle_rad << ")";
            cv::resize(debug, output, cv::Size(), 4,4, cv::INTER_NEAREST);
            cv::imshow(ss.str(), output);
            cv::waitKey(3);
        }

        std::pair<double, std::vector<double> > entry(angle_rad, data);
        costs.push_back(entry);
    }

    std::ofstream ofs((dir + "/heuristic_holo_no_obst.txt").c_str());
    ofs << dimension << '\n';
    for(std::vector<std::pair<double, std::vector<double> > >::iterator i = costs.begin(); i != costs.end(); ++i){
        ofs << i->first << '\n';
        for(std::vector<double>::iterator j = i->second.begin(); j != i->second.end(); ++j){
            ofs << *j << '\t';
        }
        ofs << '\n';
    }

    cv::waitKey(0);
}

int main(int argc, char** argv)
{
    po::options_description desc("Allowed options");
    desc.add_options()
    ("help", "show help message")
    ("dimension", po::value<int>()->required(), "width and height of the window")
    ("angles", po::value<int>()->required(), "number of discretized angle bins")
            ("directory", po::value<std::string>()->required(), "working directory")
    ;

    po::variables_map vm;

    po::positional_options_description p;
    p.add("directory", -1);

    try {
        po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);

    } catch(po::unknown_option& e) {
        std::cerr << "Error parsing parameters: " << e.what() << "\n";
        return 2;
    }

    if(vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }

    try {
        po::notify(vm);
    } catch(boost::program_options::required_option& e) {
        std::cerr << "Error parsing parameters: " << e.what() << "\n";
        return 2;
    }

    int dimension = vm["dimension"].as<int>();
    if(dimension < 0 || dimension > 1024) {
        std::cerr << "Error dimension out of range [0, 1024]: " << dimension << "\n";
    }


    int angles = vm["angles"].as<int>();
    if(angles < 1 || angles > 36) {
        std::cerr << "Error dimension out of range [1, 36]: " << angles << "\n";
    }

    precompute(vm["directory"].as<std::string>(), dimension, angles);

    return 0;
}
