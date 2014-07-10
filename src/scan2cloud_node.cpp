#include "scan2cloud.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scan2cloud");

    ScanConverter filter;

    ros::spin();

    return 0;
}
