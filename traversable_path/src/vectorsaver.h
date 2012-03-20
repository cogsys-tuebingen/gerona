#ifndef ARRAYSAVER_H
#define ARRAYSAVER_H

#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include "ros/ros.h"

/**
 * @brief Store/Load std::vector to/from a file.
 *
 * There are two methods store() and load(). store() takes a vector and stores its content to the file specified in the
 * constructor. load() is the counterpart that loads the data of such a file.
 *
 * The files have binary content. At the beginning the length of the vector is written to the file (unsigned int), then
 * the content.
 *
 * @author Felix Widmaier
 * @version 1.0
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
    // data array
    T *data = in.data();
    // length of the array
    unsigned int length = in.size();

    // write binary data to file, truncate file before writing
    std::ofstream out_file(filename_.data(), std::ios::out | std::ios::trunc | std::ios::binary);

    if (!out_file.good()) {
        ROS_ERROR("VectorSaver::store: Could not open file '%s' for writing.", filename_.data());
        return false;
    }

    // first write length of datastream
    out_file.write((char*) &length, sizeof(unsigned int));

    // ...then write data
    out_file.write((char*) data, sizeof(T) * length);

    return out_file.good();
}

template <class T>
bool VectorSaver<T>::load(std::vector<T> *out)
{
    std::ifstream in_file(filename_.data(), std::ios::in | std::ios::binary);

    if (!in_file.good()) {
        ROS_ERROR("VectorSaver::load: Could not open file '%s' for reading.", filename_.data());
        return false;
    }

    int length;
    float *data = 0;

    // first bytes of the file contain the array length
    in_file.read((char*) &length, sizeof(unsigned int));
    if (!in_file.good()) {
        ROS_ERROR("VectorSaver::load: Failed reading array length.");
        return false;
    }
    ROS_DEBUG("Number of stored range values: %d", length);

    // rest of the file are data
    data = new T[length];
    in_file.read((char*) data, sizeof(T)*length);

    if (!in_file.good()) {
        ROS_ERROR("VectorSaver::load: Failed reading data.");
        return false;
    }

    // assign data to the output-vector
    out->assign(data, data + length);

    return true;
}


#endif // ARRAYSAVER_H
