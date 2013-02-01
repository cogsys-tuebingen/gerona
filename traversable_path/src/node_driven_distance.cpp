/**
 * @brief Simple node to record driven distance of the robot from odometry.
 */
#include <ros/ros.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "driven_distance");

    tf::TransformListener tf_listener;

    ros::Rate rate(5);
    btVector3 last_pos(0,0,0);
    float distance = 0;

    while (ros::ok()) {
        ros::spinOnce();

        try {
            geometry_msgs::PointStamped point_base, point_odom;
            point_base.header.frame_id = "/base_link";
            tf_listener.transformPoint("/odom", point_base, point_odom);

            btVector3 cur_pos = btVector3(point_odom.point.x, point_odom.point.y, point_odom.point.z);
            //ROS_INFO("pos: %f,%f,%f",cur_pos[0],cur_pos[1],cur_pos[2] );

            if (last_pos.isZero()) {
                last_pos = cur_pos;
                ROS_INFO("First loop. Let's go!");
            } else {
                float step_dist = (cur_pos - last_pos).length();
                //ROS_INFO("Step: %.2f m", step_dist);

                distance += step_dist;
                ROS_INFO_THROTTLE(0.5, "Distance: %.2f m", distance);
                last_pos = cur_pos;
            }
        }
        catch (tf::TransformException e) {
            ROS_WARN_THROTTLE(1, "tf: %s", e.what());
            ROS_INFO_THROTTLE(0.5, "Distance: %.2f m", distance);
        }

        rate.sleep();
    }


    return 0;
}

