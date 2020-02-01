#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "tf2odom", ros::init_options::NoSigintHandler);

  ros::NodeHandle nh;
  ros::NodeHandle p_nh("~");

  const auto &odom_frame = p_nh.param<std::string>("odom_frame", "odom");
  const auto &base_link_frame =
      p_nh.param<std::string>("base_link_frame", "base_link");

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  auto pub = nh.advertise<nav_msgs::Odometry>("odom", 50);

  ros::Rate spin_rate(100);
  while (nh.ok()) {
    ros::spinOnce();

    geometry_msgs::TransformStamped odom_T_base_now_msg;
    geometry_msgs::TransformStamped odom_T_base_prev_msg;
    try {
      odom_T_base_now_msg =
          tf_buffer.lookupTransform(odom_frame, base_link_frame, ros::Time(0));
      odom_T_base_prev_msg = tf_buffer.lookupTransform(
          odom_frame, base_link_frame,
          odom_T_base_now_msg.header.stamp - ros::Duration(0.1));
    } catch (tf2::TransformException &ex) {
      ROS_WARN_THROTTLE(1, "%s", ex.what());
      continue;
    }

    const Eigen::Isometry3d odom_T_base_prev =
        tf2::transformToEigen(odom_T_base_prev_msg);
    const Eigen::Isometry3d odom_T_base_now =
        tf2::transformToEigen(odom_T_base_now_msg);

    const double delta_t =
        (odom_T_base_now_msg.header.stamp - odom_T_base_prev_msg.header.stamp)
            .toSec();

    const Eigen::Isometry3d base_prev_T_base_now =
        odom_T_base_prev.inverse() * odom_T_base_now;

    nav_msgs::Odometry odom;
    odom.pose.pose.position.x = odom_T_base_now_msg.transform.translation.x;
    odom.pose.pose.position.y = odom_T_base_now_msg.transform.translation.y;
    odom.pose.pose.position.z = odom_T_base_now_msg.transform.translation.z;

    odom.pose.pose.orientation.x = odom_T_base_now_msg.transform.rotation.x;
    odom.pose.pose.orientation.y = odom_T_base_now_msg.transform.rotation.y;
    odom.pose.pose.orientation.z = odom_T_base_now_msg.transform.rotation.z;
    odom.pose.pose.orientation.w = odom_T_base_now_msg.transform.rotation.w;

    if (delta_t > 0.01) {
      const auto v = base_prev_T_base_now.translation() / delta_t;
      odom.twist.twist.linear.x = v(0);
      odom.twist.twist.linear.y = v(1);
      odom.twist.twist.linear.z = v(2);

      const auto euler_angles_prev =
          odom_T_base_prev.rotation().eulerAngles(2, 1, 0);
      const auto euler_angles_now =
          odom_T_base_now.rotation().eulerAngles(2, 1, 0);

      const auto theta_prev = euler_angles_prev.x();
      const auto theta_now = euler_angles_now.x();

      const auto delta_theta = theta_now - theta_prev;
      const auto yaw = std::atan2(std::sin(delta_theta), std::cos(delta_theta));


      odom.twist.twist.angular.x = 0.0;
      odom.twist.twist.angular.y = 0.0;
      odom.twist.twist.angular.z = yaw / delta_t;
    }

    odom.header = odom_T_base_now_msg.header;
    odom.child_frame_id = odom_T_base_now_msg.child_frame_id;

    pub.publish(odom);

    spin_rate.sleep();
  }

  return 0;
}
