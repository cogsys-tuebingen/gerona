#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "odom2tf", ros::init_options::NoSigintHandler);

  ros::NodeHandle nh;
  ros::NodeHandle p_nh("~");

  const auto& odom_frame = p_nh.param<std::string>("odom_frame", "odom");
  const auto& base_link_frame = p_nh.param<std::string>("base_link_frame", "base_link");

  tf2_ros::TransformBroadcaster tf_broadcast;

  boost::function<void(const nav_msgs::Odometry::ConstPtr &)> callback =
      [&](const nav_msgs::Odometry::ConstPtr &odom) {
        geometry_msgs::TransformStamped transform;
        transform.header = odom->header;
        transform.header.frame_id = odom_frame;
        transform.child_frame_id = base_link_frame;
        transform.transform.rotation = odom->pose.pose.orientation;
        transform.transform.translation.x = odom->pose.pose.position.x;
        transform.transform.translation.y = odom->pose.pose.position.y;
        transform.transform.translation.z = odom->pose.pose.position.z;

        tf_broadcast.sendTransform(transform);
      };

  auto sub = nh.subscribe<nav_msgs::Odometry>("odom", 10, callback);

  ros::Rate spin_rate(100);
  while (nh.ok()) {
    ros::spinOnce();
    spin_rate.sleep();
  }

  return 0;
}
