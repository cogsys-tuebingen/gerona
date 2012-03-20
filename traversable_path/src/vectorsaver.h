#ifndef ARRAYSAVER_H
#define ARRAYSAVER_H

#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include "ros/ros.h"
#include "yaml-cpp/yaml.h"

/**
 * @brief Store/Load std::vector to/from a file.
 *
 * There are two methods store() and load(). store() takes a vector and stores its content to the file specified in the
 * constructor. load() is the counterpart that loads the data of such a file.
 *
 * The vectors are stored in YAML-format, so it will only work with scalar values.
 *
 * @author Felix Widmaier
 * @version 1.1
 */
template<class T>
class VectorSaver
{
public:
    /**
     * @brief Expects the filename (both for loading and storing) as argument.
     * The file will be created automaticly, if it doesn't already exist and you call store().
     *
     * @param filename Name of the file which will be used.
     */
    VectorSaver(std::string filename):
            filename_(filename) {};

    /**
     * @brief Stores a vector to the file.
     *
     * If the file already exists, it will be overwritten!
     *
     * @param in Vector that shall be written to the file,
     * @return True on success.
     */
    bool store(std::vector<T> in);

    /**
     * @brief Loads a vector from the file.
     *
     * @param out The content of the file will be written to this variable.
     * @return True on success.
     */
    bool load(std::vector<T> *out);

private:
    //! Name of the file which is used for reading/writing.
    std::string filename_;
};



//// BEGIN DEFINITION
template <class T>
bool VectorSaver<T>::store(std::vector<T> in)
{
    YAML::Emitter out;

    // convert vector to yaml-string
    out << YAML::Flow << in;

    // write data to file, truncate file before writing
    std::ofstream out_file(filename_.data(), std::ios::out | std::ios::trunc);
    if (!out_file.good()) {
        ROS_ERROR("VectorSaver::store: Could not open file '%s' for writing.", filename_.data());
        return false;
    }

    out_file << out.c_str();

    if (!out_file.good()) {
        ROS_ERROR("VectorSaver::store: Failure when writing data to file.", filename_.data());
        return false;
    }
    return true;
}

template <class T>
bool VectorSaver<T>::load(std::vector<T> *out)
{
    std::ifstream in_file(filename_.data(), std::ios::in | std::ios::binary);

    if (!in_file.good()) {
        ROS_ERROR("VectorSaver::load: Could not open file '%s' for reading.", filename_.data());
        return false;
    }

    try {
        YAML::Parser parser(in_file);
        YAML::Node doc;

        parser.GetNextDocument(doc);
        // yaml-sequence to vector
        for(YAML::Iterator it = doc.begin(); it != doc.end(); ++it) {
            T elem;
            *it >> elem;
            out->push_back(elem);
        }
    } catch(YAML::Exception &e) {
        ROS_ERROR("VectorSaver::load: YAML-Parser-Exception: %s", e.what());
        return false;
    }

    return true;
}


#endif // ARRAYSAVER_H
