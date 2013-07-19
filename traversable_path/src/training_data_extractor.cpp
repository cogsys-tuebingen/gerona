#include "training_data_extractor.h"
#include <string>

training_data_extractor::training_data_extractor()
{
    std::string classification_file;
    ros::param::get("~classification_file", classification_file);
}
