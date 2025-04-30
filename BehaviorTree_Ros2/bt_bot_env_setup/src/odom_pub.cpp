#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <random>
#include <chrono>

class RandomOdomPublisher : public rclcpp::Node
{
public:
    RandomOdomPublisher()
        : Node("random_odom_publisher")
    {
        // Publisher for the /odom topic
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        // Timer to periodically publish random odometry data
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000), // 1 second interval
            std::bind(&RandomOdomPublisher::publish_random_odom, this));
    }

private:
    void publish_random_odom()
    {
        // Generate random position and orientation values
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dist(-10.0, 10.0); // Random range between -10 and 10

        // Create the Odometry message
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = this->get_clock()->now();
        odom_msg.header.frame_id = "odom_frame";

        // Random position
        odom_msg.pose.pose.position.x = dist(gen);
        odom_msg.pose.pose.position.y = dist(gen);
        odom_msg.pose.pose.position.z = dist(gen);

        // Random orientation (quaternion)
        odom_msg.pose.pose.orientation.x = dist(gen);
        odom_msg.pose.pose.orientation.y = dist(gen);
        odom_msg.pose.pose.orientation.z = dist(gen);
        odom_msg.pose.pose.orientation.w = 1.0; // Keep w fixed for simplicity

        // Optionally, we can also randomize the velocity (if needed)
        odom_msg.twist.twist.linear.x = dist(gen);
        odom_msg.twist.twist.linear.y = dist(gen);
        odom_msg.twist.twist.linear.z = dist(gen);

        odom_msg.twist.twist.angular.x = dist(gen);
        odom_msg.twist.twist.angular.y = dist(gen);
        odom_msg.twist.twist.angular.z = dist(gen);

        // Publish the message
        // RCLCPP_INFO(this->get_logger(), "Publishing random odom data: position [%.2f, %.2f, %.2f] | orientation [%.2f, %.2f, %.2f, %.2f]",
        //             odom_msg.pose.pose.position.x,
        //             odom_msg.pose.pose.position.y,
        //             odom_msg.pose.pose.position.z,
        //             odom_msg.pose.pose.orientation.x,
        //             odom_msg.pose.pose.orientation.y,
        //             odom_msg.pose.pose.orientation.z,
        //             odom_msg.pose.pose.orientation.w);
        
        odom_publisher_->publish(odom_msg);
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RandomOdomPublisher>());
    rclcpp::shutdown();
    return 0;
}
