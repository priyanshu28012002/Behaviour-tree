#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class KalmanOdomNode : public rclcpp::Node
{
public:
  KalmanOdomNode() : Node("kalman_odom_node")
  {
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&KalmanOdomNode::odom_callback, this, std::placeholders::_1));

    filtered_odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/filtered_odom", 10);
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/filtered_pose", 10);

    // Initialize Kalman filter variables
    x_est_ = 0.0;
    y_est_ = 0.0;
    vx_est_ = 0.0;
    vy_est_ = 0.0;

    RCLCPP_INFO(this->get_logger(), "Kalman Odom Node has been started.");
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    double x_meas = msg->pose.pose.position.x;
    double y_meas = msg->pose.pose.position.y;

    // Kalman filter (simplified 2D linear position-only update)
    double K = 0.5;  // Kalman gain (tunable)

    x_est_ = x_est_ + K * (x_meas - x_est_);
    y_est_ = y_est_ + K * (y_meas - y_est_);

    // Publish filtered odometry
    auto filtered_odom = *msg;  // Copy original for timestamp/frame_id
    filtered_odom.pose.pose.position.x = x_est_;
    filtered_odom.pose.pose.position.y = y_est_;
    filtered_odom_publisher_->publish(filtered_odom);

    // Publish pose stamped
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = msg->header;
    pose_msg.pose = filtered_odom.pose.pose;
    pose_publisher_->publish(pose_msg);

    RCLCPP_INFO(this->get_logger(), "Filtered pose: x=%.2f, y=%.2f", x_est_, y_est_);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr filtered_odom_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;

  // Kalman filter state (position only for simplicity)
  double x_est_, y_est_;
  double vx_est_, vy_est_;  // Not used here but can be extended
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KalmanOdomNode>());
  rclcpp::shutdown();
  return 0;
}
