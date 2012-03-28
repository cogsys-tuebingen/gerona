#include "pathfollower.h"

PathFollower::PathFollower()
{
    subscribe_scan_classification_ = node_handle_.subscribe("/scan/traversability", 100,
                                                            &PathFollower::scan_classification_callback, this);
}

void PathFollower::scan_classification_callback(traversable_path::LaserScanClassificationConstPtr scan)
{
    int path;

    // search traversable area in front of the robot (assuming, "in front" is approximalty in the middle of the scan)
    unsigned int mid = scan->traversable.size() / 2;

    // search traversable area
    unsigned int beginning = 0, end = 0;
    for (unsigned int i = 0; i < mid; ++i) {
        if (scan->traversable[mid+i]) {
            beginning = mid+i;
            end = mid+i;
            break;
        } else if (scan->traversable[mid-i]) {
            beginning = mid-i;
            end = mid-i;
            break;
        }
    }

    if (beginning == 0) {
        ROS_INFO("No traversable paths found.");
        // TODO: stop robot
        return;
    }

    // get range of the area
    while (beginning > 0 && scan->traversable[beginning]) {
        --beginning;
    }
    while (end < scan->traversable.size() && scan->traversable[end]) {
        ++end;
    }

    path = beginning + (end-beginning)/2 - mid;

    // TODO: command robot depending on path. path = 0: nothing to do. < 0: navigate left. > 0: navigate right.
}

//--------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "follow_path");

    PathFollower follower;

    // main loop
    ros::spin();
    return 0;
}
