#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

class JoyServiceNode : public rclcpp::Node
{
public:
  JoyServiceNode() : Node("joy_service_node")
  {
    // Subscriber to the /joy topic
    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&JoyServiceNode::joy_callback, this, std::placeholders::_1));

    // Publisher for the /cmd_vel topic
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    
    // Create a timer to publish cmd_vel at a fixed rate (e.g., 10 Hz)
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),  // 100 ms = 10 Hz
      std::bind(&JoyServiceNode::publish_cmd_vel, this)
    );
    
    RCLCPP_INFO(this->get_logger(), "Joy service node has been started.");
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // Store joystick axes for later use
    joystick_axes_ = *msg;
    start = true;
  }

  void publish_cmd_vel()
  {
    // Create a Twist message for cmd_vel

    if(start){
        geometry_msgs::msg::Twist twist;
    
    // Map joystick axes to linear and angular velocities
    twist.linear.x = joystick_axes_.axes[1] * 0.5;  // Typically Y-axis controls forward/backward movement
    twist.angular.z = joystick_axes_.axes[2] * 0.5; // Typically X-axis controls rotation

    // Publish the cmd_vel message
    cmd_vel_publisher_->publish(twist);
    
    RCLCPP_INFO(this->get_logger(), "Published: linear.x = %f, angular.z = %f", twist.linear.x, twist.angular.z);
    }
    
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::TimerBase::SharedPtr timer_; // Timer for periodic publishing
  sensor_msgs::msg::Joy joystick_axes_; // Store joystick axes values
  bool start  =false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyServiceNode>());
  rclcpp::shutdown();
  return 0;
}