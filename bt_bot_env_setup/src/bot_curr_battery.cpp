#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <chrono>
#include <thread>

class BatteryStatus : public rclcpp::Node {
public:
    BatteryStatus() : Node("battery_status_publisher"), battery_status_val(100.0), last_x(0.0), last_y(0.0)
    {
        battery_status_pub_ = this->create_publisher<std_msgs::msg::Float32>("/battery_status", 10);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&BatteryStatus::odom_callback, this, std::placeholders::_1)
        );

        battery_status_pub_timer_ = this->create_wall_timer(
            std::chrono::microseconds(1),
            std::bind(&BatteryStatus::publish_battery_status, this)
        );

        battery_status_modify_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&BatteryStatus::modify_battery_status, this)
        );
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;

        last_x = x;
        last_y = y;
    }

    void modify_battery_status() {
        if (std::abs(last_x) < 0.2 && std::abs(last_y) < 0.2) {
            battery_status_val += 10.0;
            if (battery_status_val > 100.0) {
                battery_status_val = 100.0;
            }
        } else {
            battery_status_val -= 1.0;
            if (battery_status_val < 0.0) {
                battery_status_val = 0.0;
            }
        }

        // RCLCPP_INFO(this->get_logger(), "Battery status after modification: %.2f", battery_status_val);
    }

    void publish_battery_status() {
        std_msgs::msg::Float32 battery_status_msg;
        battery_status_msg.data = battery_status_val;

        battery_status_pub_->publish(battery_status_msg);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_status_pub_;
    rclcpp::TimerBase::SharedPtr battery_status_pub_timer_;
    rclcpp::TimerBase::SharedPtr battery_status_modify_timer_;
    float battery_status_val;
    double last_x, last_y;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BatteryStatus>());
    rclcpp::shutdown();
    return 0;
}