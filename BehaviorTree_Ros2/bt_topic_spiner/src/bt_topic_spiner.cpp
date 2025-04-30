#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "bt_topic_spiner/bt_topic_spiner.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "rclcpp/node_options.hpp"
#include <memory.h>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<GenericSpinnerNode>(options);   
  
  node->add_topic<std_msgs::msg::Float32MultiArray>("risk");
  node->add_topic<std_msgs::msg::Float32>("battery_status");
  node->add_topic<nav_msgs::msg::Odometry>("odom");
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}