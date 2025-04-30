#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <random>

class RiskPublisherNode : public rclcpp::Node
{
public:
    RiskPublisherNode() : Node("risk_publisher_node")
    {
        risk_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/risk", 10);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&RiskPublisherNode::publishRiskData, this));

        std::random_device rd;
        rng_ = std::mt19937(rd());
        dist_ = std::uniform_real_distribution<float>(0.0, 1.0);
    }

private:
    void publishRiskData()
    {
        float x = dist_(rng_);

        int integer_part = static_cast<int>(x);
        float fractional_part = x - integer_part;

        int first_decimal = static_cast<int>(fractional_part * 10) % 10;
        int second_decimal = static_cast<int>(fractional_part * 100) % 10;

        if (first_decimal % 2 == 0 && second_decimal % 2 == 0)
        {
            n_even++;
        }

        if (first_decimal % 2 != 0 && second_decimal % 2 != 0)
        {
            n_odd++;
        }

        if (n_odd == control_pub || n_even == control_pub)
        {
            n_odd = 0;
            n_even = 0;
            std_msgs::msg::Float32MultiArray msg;
            msg.data.resize(3);

            for (auto &val : msg.data)
            {
                val = dist_(rng_);
            }
            risk_publisher_->publish(msg);
        }
    }

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr risk_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::mt19937 rng_;
    std::uniform_real_distribution<float> dist_;
    int n_even = 0;
    int n_odd = 0;
    int control_pub = 5;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RiskPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
