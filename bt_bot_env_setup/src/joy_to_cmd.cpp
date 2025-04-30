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
    // Create a Twist message for cmd_vel
    if (msg->buttons[0] == 1) {
      angular_vel_ += 0.01;
    }
    if (msg->buttons[1] == 1) {
      linear_vel_ -= 0.01;
    }
    if (msg->buttons[2] == 1) {
      angular_vel_ -= 0.01;
    }
    if (msg->buttons[3] == 1) {
      linear_vel_ += 0.01;
    }
    start = true;
  }

  void publish_cmd_vel()
  {
    
    if(start){
        geometry_msgs::msg::Twist twist;
    
    // Map joystick axes to linear and angular velocities
    twist.linear.x = joystick_axes_.axes[1] * linear_vel_;  // Typically Y-axis controls forward/backward movement
    twist.angular.z = joystick_axes_.axes[2] * angular_vel_; // Typically X-axis controls rotation

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
  double linear_vel_;
  double angular_vel_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyServiceNode>());
  rclcpp::shutdown();
  return 0;
}

// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/joy.hpp"
// #include "geometry_msgs/msg/twist.hpp"

// class JoyServiceNode : public rclcpp::Node
// {
// public:
//   JoyServiceNode() : Node("joy_service_node"), linear_vel_(0.0), angular_vel_(0.0)
//   {
//     joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
//       "/joy", 10, std::bind(&JoyServiceNode::joy_callback, this, std::placeholders::_1));

//     cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

//     timer_ = this->create_wall_timer(
//       std::chrono::milliseconds(100),  // 10 Hz
//       std::bind(&JoyServiceNode::publish_cmd_vel, this));

//     RCLCPP_INFO(this->get_logger(), "Joy service node with button control has started.");
//   }

// private:
//   void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
//   {

//     if (msg->buttons[0] == 1) {
//       angular_vel_ += 0.1;
//     }
//     if (msg->buttons[1] == 1) {
//       linear_vel_ -= 0.1;
//     }
//     if (msg->buttons[2] == 1) {
//       angular_vel_ -= 0.1;
//     }
//     if (msg->buttons[3] == 1) {
//       linear_vel_ += 0.1;
//     }

//     // Clamp values if needed (optional safety limits)
//     linear_vel_ = std::clamp(linear_vel_, -1.0, 1.0);
//     angular_vel_ = std::clamp(angular_vel_, -1.0, 1.0);
//   }

//   void publish_cmd_vel()
//   {
//     geometry_msgs::msg::Twist twist;
//     twist.linear.x = linear_vel_;
//     twist.angular.z = angular_vel_;
//     cmd_vel_publisher_->publish(twist);

//     RCLCPP_INFO(this->get_logger(), "Published: linear.x = %f, angular.z = %f", twist.linear.x, twist.angular.z);
//   }

//   rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
//   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
//   rclcpp::TimerBase::SharedPtr timer_;

//   double linear_vel_;
//   double angular_vel_;
// };

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<JoyServiceNode>());
//   rclcpp::shutdown();
//   return 0;
// }
