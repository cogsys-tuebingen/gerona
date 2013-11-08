/**
 * \brief Trains the classifier
 *
 * This node trains the classifier and saves the learned parameters to a file, which will be used by classify_terrain.
 * Obviously, the trainer has to be called before classification can be done.
 *
 * Usage: rosrun traversable_path trainer path/to/training_data.csv
 *
 * The training_data.csv has to be formated as follows:
 *  - one line per sample
 *  - values are separated by commas
 *  - the class label is assumed to be located in the last column
 *  - the csv-parser by OpenCV is very lame, thus the file must not contain comments or empty lines!
 *
 */

#include <iostream>
#include <fstream>
#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/ml/ml.hpp>

using namespace std;


bool loadTrainingData(char* filename, CvMLData *output)
{
    if ( output->read_csv(filename) ) {
        return false;
    }

    // the last column contains the class label
    output->set_response_idx( output->get_values()->cols-1 );

    ROS_DEBUG("Assume column %d to be the class label.", output->get_response_idx());
    //ROS_INFO_STREAM(cv::Mat(output->get_responses()));

//    CvTrainTestSplit split(0.66f);
//    output->set_train_test_split(&split);
    /** \todo parameter for mixing */
    //output->mix_train_and_test_idx();

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trainer");
    ros::NodeHandle nh;

    const std::string CLASSIFIER_FILE = ros::package::getPath(ROS_PACKAGE_NAME) + std::string("/classifier_params.yaml");

    //ROS_INFO("# Arguments: %d", argc);
    //copy(argv, argv+argc, ostream_iterator<string>(cout, "|"));

    // check parameter count
    if (argc < 2) {
        ROS_ERROR("Insufficient arguments.\nUsage: trainer <traindata-csv-file>");
        return 1;
    }


    CvMLData training_data;

    if (loadTrainingData(argv[1], &training_data)) {
        ROS_DEBUG("Loaded training data from %s", argv[1]);
    } else {
        ROS_FATAL("Failed at reading file %s. Exit.", argv[1]);
        return 2;
    }


    CvRTrees classifier;

    ROS_DEBUG("Start training...");
    classifier.train(&training_data);
    ROS_DEBUG("...finished.");

    ROS_DEBUG("Store parameters to file %s", CLASSIFIER_FILE.c_str());
    classifier.save(CLASSIFIER_FILE.c_str());


    ROS_INFO("Train error: %g", classifier.calc_error(&training_data, CV_TRAIN_ERROR));
    ROS_INFO("Test error:  %g", classifier.calc_error(&training_data, CV_TEST_ERROR));

    return 0;
}

