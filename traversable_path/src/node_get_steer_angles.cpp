/**
 * @brief Node that logs steer angles for evaluation.
 *
 * Subscribes to /ramaxx_cmd which publishes the velocity and steer angles of front and rear axes
 * and writes them to a file.
 *
 * @author Felix Widmaier
 * @version $Id$
 */
#include <vector>
#include <fstream>
#include <ros/ros.h>
#include <ramaxxbase/RamaxxMsg.h>

using namespace std;

ofstream g_out_file;

//! Callback
void callback(const ramaxxbase::RamaxxMsgConstPtr &msg)
{
    float steer_front = 0.0, steer_rear = 0.0, speed = 0.0;
    int field_counter = 0;

    // get values
    for (vector<ramaxxbase::key_val_pair>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it) {
        switch (it->key) {
        case ramaxxbase::RamaxxMsg::CMD_STEER_FRONT_DEG:
            steer_front = it->value;
            ++field_counter;
            break;

        case ramaxxbase::RamaxxMsg::CMD_STEER_REAR_DEG:
            steer_rear = it->value;
            ++field_counter;
            break;

        case ramaxxbase::RamaxxMsg::CMD_SPEED:
            speed = it->value;
            ++field_counter;
            break;
        }
    }

    // only continue if all three values where set
    if (field_counter == 3) {
        g_out_file << speed << "\t" << steer_front << "\t" << steer_rear << endl;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "display_path");
    ros::NodeHandle node_handle;

    if (argc <= 1) {
        ROS_FATAL("Usage: get_steer_angles <outputfile>");
        return 1;
    }

    g_out_file.open(argv[1]);
    if (!g_out_file.is_open()) {
        ROS_FATAL("Could not oben file");
        return 1;
    }
    g_out_file << "#speed\tfront\trear" << endl;

    // subscribe for /ramaxx_cmd
    ros::Subscriber sub = node_handle.subscribe("/ramaxx_cmd", 0, &callback);

    ros::Rate rate(5);
    while (ros::ok()) {
        rate.sleep();
        ros::spinOnce();
    }

    g_out_file.close();

    return 0;
}
