#include "scan2cloud.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scan2cloud", ros::init_options::NoSigintHandler);

    ScanConverter filter;

    filter.spin();

    return 0;
}
