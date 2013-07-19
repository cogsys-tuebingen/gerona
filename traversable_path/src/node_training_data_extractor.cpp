#include <ros/ros.h>
#include "training_data_extractor.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "training_data_extractor");
    ros::NodeHandle nh;

    TrainingDataExtractor tde;

    tde.generateSamples();

    return 0;
}
