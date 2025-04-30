#pragma once

#include <rclcpp/rclcpp.hpp>
// Standard messages
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/char.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/u_int64.hpp>

// Geometry messages
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/wrench.hpp>

// Nav messages
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

// Sensor messages
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/range.hpp>

// Visualization messages
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// Builtin types
#include <builtin_interfaces/msg/duration.hpp>
#include <builtin_interfaces/msg/time.hpp>

// Diagnostic messages (optional)
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <chrono>
#include <string>
#include <type_traits>
using namespace std::chrono_literals;

class GenericSpinnerNode : public rclcpp::Node
{
public:
  explicit GenericSpinnerNode(const rclcpp::NodeOptions & options)
  : Node("bt_topic_spiner_node" , options)
  {
    timer_ = this->create_wall_timer(
      100ms, std::bind(&GenericSpinnerNode::timer_callback, this));
      
  }

  template<typename T>
  void add_topic(const std::string& topic_name)
  {
    // Create subscription
    auto subscription = this->create_subscription<T>(
      topic_name, 10,
      [this, topic_name](const typename T::SharedPtr msg) {
        this->topic_callback<T>(topic_name, msg);
      });
    
    // Create publisher and handler with default message
    auto handler = std::make_shared<TypedTopicHandler<T>>();
    handler->publisher = this->create_publisher<T>("bt/" + topic_name, 10);
    
    // Initialize with default message
    handler->last_msg = std::make_shared<T>();
    initialize_default_message<T>(*handler->last_msg);
    
    topic_handlers_[topic_name] = handler;
    subscriptions_.push_back(subscription);
  }

private:
  // Helper function to initialize default messages
  template<typename T>
  void initialize_default_message(T& msg) {
    // Default implementation does nothing (for types that don't need initialization)
  }

  // Specialization for std_msgs::msg::String
  void initialize_default_message(std_msgs::msg::String& msg) {
    msg.data = "";  // Initialize with empty string
  }

  // Specialization for std_msgs::msg::Int32
  void initialize_default_message(std_msgs::msg::Int32& msg) {
    msg.data = 0;  // Initialize with zero
  }

  struct TopicHandlerBase {
    virtual ~TopicHandlerBase() = default;
    virtual void publish() = 0;
  };

  template<typename T>
  struct TypedTopicHandler : public TopicHandlerBase {
    typename rclcpp::Publisher<T>::SharedPtr publisher;
    typename T::SharedPtr last_msg;

    void publish() override {
      if (last_msg) {
        publisher->publish(*last_msg);
      }
    }
  };

  template<typename T>
  void topic_callback(const std::string& topic_name, const typename T::SharedPtr msg)
  {
    auto handler = std::static_pointer_cast<TypedTopicHandler<T>>(topic_handlers_[topic_name]);
    handler->last_msg = msg;
  }

  void timer_callback()
  {
    for (auto& pair : topic_handlers_) {
      pair.second->publish();
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  std::unordered_map<std::string, std::shared_ptr<TopicHandlerBase>> topic_handlers_;
};