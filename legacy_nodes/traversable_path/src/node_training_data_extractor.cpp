/**
 * @brief This node generates training samples from bagfiles.
 *
 * Input are a bagfile and a classification file. The bagfile has to contain a few seconds of scans while the robot
 * is *not* moving. The classification file is a simple comma separated text file that defines the angular intervals
 * where there are path borders in the scan (see TrainingDataExtractor::loadClassifications for a description of the
 * structure of this file).
 * The output is an ARFF-file that contains the samples in the format used by Weka.
 *
 * Node Parameters:
 *  ~sample_size            Size of the samples (= window around the "main" point of the sample).
 *  ~calibration_file       File that contains scanner calibration data (same as for classify_terrain).
 *  ~bagfile                Path to the bagfile.
 *  ~classification_file    Path to the classification file.
 *  ~output_file            Output file. This will overwrite existing files! Default: "tp_training_data.arff".
 *  ~print_header           If set to false, the ARFF files will have no header (for concatenating files). Default: true
 */
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
