/**

    @year   2015
    @author Karsten Bohlmann
    @email  bohlmann@gmail.com
    @file   course_planner_node.cpp


*/
#include <math.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include "course_planner.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "course_planner", ros::init_options::NoSigintHandler);

    CoursePlanner planner;

    ros::WallRate r(2);
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();

        planner.tick();
    }
}
