#ifndef CALIBRATIONDATASTORAGE_H
#define CALIBRATIONDATASTORAGE_H

#include <vector>
#include <string>

/**
 * @brief Abstraction Layer to store and load laser calibration data to/from a file.
 *
 * The calibration data of one layer consists of the range values of a preferably perfect plane. Since there are
 * multiple layers, we have to store an array of arrays of floats.
 *
 * @author Felix Widmaier
 * @version $Id$
 */
class CalibrationDataStorage
{
public:
    CalibrationDataStorage(std::string filename) :
        filename_(filename) {}

    /**
     * @brief Stores the calibration data to the file.
     *
     * If the file already exists, it will be overwritten!
     *
     * @param One vector of float values (range-scan) for each layer, bundled in a vector.
     * @return True on success.
     */
    bool store(std::vector<std::vector<float> > data);

    /**
     * @brief Loads the calibration data from the file.
     *
     * @param out Output parameter. One vector of float values (range-scan) for each layer, bundled in a vector.
     * @return True on success.
     */
    bool load(std::vector<std::vector<float> > *out);

private:
    //! Name of the file which is used for reading/writing.
    std::string filename_;
};

#endif // CALIBRATIONDATASTORAGE_H
