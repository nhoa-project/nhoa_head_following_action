#include <nhoa_head_following_action/nhoa_head_following_action.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "nhoa_head_following_action");

  ROS_INFO("Starting nhoa_head_following_action...");
  // tf::TransformListener tf(ros::Duration(10));
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  Nhoa_head_following_action headFollowing(&tfBuffer);

  ros::spin();

  return 0;
}
