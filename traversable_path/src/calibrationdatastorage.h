#ifndef CALIBRATIONDATASTORAGE_H
#define CALIBRATIONDATASTORAGE_H

#include <vector>
#include <string>

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
     * @param
     * @return True on success.
     */
    bool store(std::vector<std::vector<float> > data);

    /**
     * @brief Loads the calibration data from the file.
     *
     * @param out The content of the file will be written to this variable.
     * @return True on success.
     */
    bool load(std::vector<std::vector<float> > *out);

private:
    //! Name of the file which is used for reading/writing.
    std::string filename_;
};

#endif // CALIBRATIONDATASTORAGE_H
