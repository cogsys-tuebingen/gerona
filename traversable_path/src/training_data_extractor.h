#ifndef TRAINING_DATA_EXTRACTOR_H
#define TRAINING_DATA_EXTRACTOR_H

#include <ros/ros.h>

class training_data_extractor
{
public:
    training_data_extractor();

private:
    struct Interval
    {
        float from, to;
    };
};

#endif // TRAINING_DATA_EXTRACTOR_H
