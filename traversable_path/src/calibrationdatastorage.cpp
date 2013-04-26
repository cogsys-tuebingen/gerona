#include "calibrationdatastorage.h"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <ros/ros.h>

bool CalibrationDataStorage::store(std::vector<std::vector<float> > data)
{
    // create yaml
    YAML::Emitter calib;
    calib << YAML::BeginSeq;

    for (size_t i = 0; i < data.size(); ++i) {
        calib << YAML::Flow << data[i];
    }
    calib << YAML::EndSeq;

    // write yaml-data to file, truncate file before writing
    std::ofstream out_file(filename_.data(), std::ios::out | std::ios::trunc);
    if (!out_file.good()) {
        ROS_ERROR("CalibrationDataStorage::store: Could not open file '%s' for writing.", filename_.data());
        return false;
    }

    out_file << calib.c_str();

    if (!out_file.good()) {
        ROS_ERROR("CalibrationDataStorage::store: Failure when writing data to file '%s'.", filename_.data());
        return false;
    }
    return true;
}

bool CalibrationDataStorage::load(std::vector<std::vector<float> > *out)
{
    std::ifstream in_file(filename_.data(), std::ios::in | std::ios::binary);

    if (!in_file.good()) {
        ROS_ERROR("CalibrationDataStorage::load: Could not open file '%s' for reading.", filename_.data());
        return false;
    }

    try {
        YAML::Parser parser(in_file);
        YAML::Node doc;

        parser.GetNextDocument(doc);
        // the outer loop iterates over the layers
        for (YAML::Iterator layer_it = doc.begin(); layer_it != doc.end(); ++layer_it) {
            std::vector<float> scan;
            // the inner loop iterates over the range values of the current layer (seems like there is no direct
            // conversion to std::vector like it is for the other way round in store()).
            for (YAML::Iterator scan_it = layer_it->begin(); scan_it != layer_it->end(); ++scan_it) {
                float elem;
                *scan_it >> elem;
                scan.push_back(elem);
            }
            out->push_back(scan);
        }
    } catch(YAML::Exception &e) {
        ROS_ERROR("CalibrationDataStorage::load: File: '%s' YAML-Parser-Exception: %s", filename_.data(), e.what());
        return false;
    }

    return true;
}
