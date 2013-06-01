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

void printProgress(int angle, int angles, char symbol = 'X')
{
    std::cout << '\r';
    std::cout << "progress: [";
    int i = 0;
    for(; i < angle-1; ++i)
        std::cout << 'X';
    std::cout << symbol;
    ++i;
    for(; i < angles-1; ++i)
        std::cout << " ";
    std::cout << "]" << std::flush;
}

void precompute(const std::string& dir, int dimension, int angles, double res, bool debug)
{

    lib_path::CurveGenerator generator;

    //////// CSC
    generator.parse("LSL");
    generator.parse("LSR");
    generator.parse("RSL");
    generator.parse("RSR");
    //    generator.parse("|LSL");
    //    generator.parse("|LSR");
    //    generator.parse("|RSL");
    //    generator.parse("|RSR");
    //    generator.parse("|L|SL");
    //    generator.parse("|L|SR");
    //    generator.parse("|R|SL");
    //    generator.parse("|R|SR");
    //    generator.parse("LS|L");
    //    generator.parse("LS|R");
    //    generator.parse("RS|L");
    //    generator.parse("RS|R");
    //    generator.parse("L|SL");
    //    generator.parse("L|SR");
    //    generator.parse("R|SL");
    //    generator.parse("R|SR");
    //    generator.parse("L|S|L");
    //    generator.parse("L|S|R");
    //    generator.parse("R|S|L");
    //    generator.parse("R|S|R");
    //    generator.parse("|LS|L");
    //    generator.parse("|LS|R");
    //    generator.parse("|RS|L");
    //    generator.parse("|RS|R");
    //    generator.parse("|L|S|L");
    //    generator.parse("|L|S|R");
    //    generator.parse("|R|S|L");
    //    generator.parse("|R|S|R");

    ///// CCC
    generator.parse("LRL");
    generator.parse("RLR");
    //    generator.parse("L|R|L");
    //    generator.parse("R|L|R");
    //    generator.parse("|L|R|L");
    //    generator.parse("|R|L|R");
    //    generator.parse("LR|L");
    //    generator.parse("RL|R");
    //    generator.parse("|LR|L");
    //    generator.parse("|RL|R");
    //    generator.parse("L|RL");
    //    generator.parse("R|LR");
    //    generator.parse("|L|RL");
    //    generator.parse("|R|LR");
    //    generator.parse("LR(b)|L(b)R");
    //    generator.parse("|LR(b)|L(b)R");
    //    generator.parse("RL(b)|R(b)L");
    //    generator.parse("|RL(b)|R(b)L");
    //    generator.parse("L|R(b)L(b)|R");
    //    generator.parse("R|L(b)R(b)|L");
    //    generator.parse("|L|R(b)L(b)|R");
    //    generator.parse("|R|L(b)R(b)|L");

    double curve_rad = 2.5; // m

    double anglestep = 2 * M_PI / angles;

    Pose2d target(dimension / 2., dimension / 2., 0);

    SimpleGridMap2d map_info(dimension, dimension, res);
    map_info.setOrigin(Point2d(0, 0));
    map_info.setLowerThreshold(20);
    map_info.setUpperThreshold(50);

    generator.set_circle_radius(curve_rad);
    generator.set_cost_backwards(20.0);
    generator.set_cost_curve(1.0);
    generator.set_cost_backwards(1.0);
    generator.set_cost_forwards(1.0);
    generator.set_cost_straight(1.0);
    generator.set_use_map_cost(false);

    std::vector<std::pair<double, std::vector<double> > > costs;

    for(int angle = 0; angle < angles; ++angle) {
        double angle_rad = angle * anglestep;

        double min_cost = NOT_FREE;
        double max_cost = 0;

        std::vector<double> data(dimension*dimension);

        cv::Mat debug_img;
        if(debug) {
            debug_img = cv::Mat(dimension, dimension, CV_8UC3, cv::Scalar::all(128));
        }

        printProgress(angle, angles);

        int i = 0;
        int padding = 10;
        for(int y = 0; y < dimension; ++y) {
            if((y % 10) == 0) {
                printProgress(angle, angles, ((y % 20) == 0) ? '\\' : '/');
            }
            for(int x = 0; x < dimension; ++x) {
                Pose2d start(x,y, angle_rad);
                Curve* c = generator.find_path(start, target, &map_info, true);

                double v = c->weight();
//                v = hypot(target.x-x, target.y-y);

                delete c;

                if(v < NOT_FREE) {
                    double fx = 1.0;
                    double fy = 1.0;

                    if(x < padding) {
                        fx = x / double(padding);
                    } else if(x >= dimension - padding) {
                        fx = (-x + dimension) / double(padding);
                    }

                    if(y < padding) {
                        fy = y / double(padding);
                    } else if(y >= dimension - padding) {
                        fy = (-y + dimension) / double(padding);
                    }

//                    double f = std::min(fx, fy);

//                    double euclid_dist = hypot(x - target.x, y - target.y);
//                    data[i++] = f * v + (1.0 - f) * euclid_dist;

                    data[i++] = v;

                    max_cost = std::max(v, max_cost);
                    min_cost = std::min(v, min_cost);
                } else {
                    data[i++] = 0;
                }
            }

            if(debug){
                for(int yy = 0; yy < dimension; ++yy) {
                    unsigned ws = yy * dimension;
                    for(int xx = 0; xx < dimension; ++xx) {
                        debug_img.at<cv::Vec3b>(yy,xx) = cv::Vec3b::all(255 * (data[ws + xx] - min_cost) / (max_cost - min_cost));
                    }
                }

                std::stringstream ss;
                cv::Mat output;
                ss << "computing (angle=" << angle_rad << ")";

                cv::resize(debug_img, output, cv::Size(), 4,4, cv::INTER_NEAREST);
                cv::imshow(ss.str(), output);
                cv::waitKey(3);
            }
        }

        std::pair<double, std::vector<double> > entry(angle_rad, data);
        costs.push_back(entry);
    }

    std::ofstream ofs((dir + "/heuristic_holo_no_obst.txt").c_str());
    ofs << dimension << '\n';
    ofs << angles << '\n';
    ofs << res << '\n';
    for(std::vector<std::pair<double, std::vector<double> > >::iterator i = costs.begin(); i != costs.end(); ++i) {
        ofs << i->first << '\n';
        for(std::vector<double>::iterator j = i->second.begin(); j != i->second.end(); ++j) {
            ofs << *j << ' ';
        }
        ofs << '\n';
    }

    printProgress(angles, angles);
    std::cout << std::endl << "done" << std::endl;

    cv::waitKey(0);
}

int main(int argc, char** argv)
{
    po::options_description desc("Allowed options");
    desc.add_options()
            ("help", "show help message")
            ("dimension", po::value<int>()->required(), "width and height in m")
            ("angles", po::value<int>()->required(), "number of discretized angle bins")
            ("resolution", po::value<double>()->required(), "resolution (^= width of one cell in [m])")
            ("directory", po::value<std::string>()->required(), "working directory")
            ("debug", "debug (display images)")
            ;

    // --dimension=16 --angles=16 --resolution=0.5 --directory=

    po::variables_map vm;

    po::positional_options_description p;
    p.add("directory", -1);


    try {
        po::parsed_options parsed = po::parse_command_line(argc, argv, desc, po::command_line_style::unix_style);
        po::store(parsed, vm);

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

    int dimension_in_m = vm["dimension"].as<int>();
    if(dimension_in_m < 0 || dimension_in_m > 100) {
        std::cerr << "Error dimension out of range [0, 100]: " << dimension_in_m << "\n";
        return 2;
    }


    int angles = vm["angles"].as<int>();
    if(angles < 1 || angles > 36) {
        std::cerr << "Error angles out of range [1, 36]: " << angles << "\n";
        return 2;
    }


    double resolution = vm["resolution"].as<double>();
    if(resolution <= 0.0 || resolution > 2.0) {
        std::cerr << "Error resolution out of range ]0.0, 2.0]: " << resolution << "\n";
        return 2;
    }

    bool debug = vm.count("debug");

    int dimension_in_pixels = dimension_in_m / resolution;
    precompute(vm["directory"].as<std::string>(), dimension_in_pixels, angles, resolution, debug);

    return 0;
}
