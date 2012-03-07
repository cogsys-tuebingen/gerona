#include "ros/ros.h"
#include "ramaxxbase/PTZ.h"
#include <stdio.h>

using namespace std;

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "movecam");
  ros::NodeHandle n;

  ros::Publisher ptz_pub = n.advertise<ramaxxbase::PTZ>("cmd_ptz", 1000);

  ros::Rate loop_rate(10);

  float pan = 0.0;
  float tilt = 0.0;
  char c;

  cout << "movecam ready" << endl;

  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    ramaxxbase::PTZ msg;


    c = getchar();

    switch (c) {
    case 'w':
        tilt -= 0.1;
        break;
    case 's':
        tilt += 0.1;
        break;
    case 'a':
        pan += 0.1;
        break;
    case 'd':
        pan -= 0.1;
        break;
    }

    msg.pan = pan;
    msg.tilt = tilt;

    ROS_INFO("move cam");

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    ptz_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
