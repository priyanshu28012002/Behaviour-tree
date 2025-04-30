#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_ros2/bt_action_node.hpp>
#include <iostream>
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp_v3/basic_types.h"
#include "behaviortree_cpp_v3/tree_node.h"

#include "std_msgs/msg/string.hpp"
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "geometry_msgs/msg/twist.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <mutex>

using namespace std::chrono_literals;
using std::chrono::milliseconds;
using std::placeholders::_1;
std::atomic_bool switchActive{true};
using namespace BT;

class FloatThresholdDecorator : public BT::DecoratorNode {
    public:
        FloatThresholdDecorator(const std::string& name, const BT::NodeConfiguration& config)
            : BT::DecoratorNode(name, config) {
        }
    
        static BT::PortsList providedPorts() {
            return {
                BT::InputPort<float>("threshold", 0.5f, "Threshold value"),
                BT::InputPort<float>("value", "Value to compare against threshold")
            };
        }
    
    private:
        BT::NodeStatus tick() override {
            float threshold, value;
            
            if (!getInput("threshold", threshold)) {
                throw BT::RuntimeError("missing required input [threshold]");
            }
            
            if (!getInput("value", value)) {
                throw BT::RuntimeError("missing required input [value]");
            }
    
            if (value > threshold) {
                // Execute child node
                const BT::NodeStatus child_status = child_node_->executeTick();
                return child_status;
            }
            
            return BT::NodeStatus::FAILURE;
        }
    };

class ReadFloat : public BT::AsyncActionNode, public rclcpp::Node {


public:
    ReadFloat(const std::string &name, const BT::NodeConfiguration &config)
        : BT::AsyncActionNode(name, config), Node("ReadFloat_node") {
        
        auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
        subscription_ = this->create_subscription<std_msgs::msg::Float32>(
            "/value", sensor_qos,
            [&](const std_msgs::msg::Float32::SharedPtr msg) {
                topic_callback(msg);
            });

        publisher_ = this->create_publisher<std_msgs::msg::Float32>("published_value", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000), std::bind(&ReadFloat::timer_callback, this));
    }

    virtual void halt() override {
        RCLCPP_INFO(this->get_logger(), "HALT IS CALLING ");
    }

    BT::NodeStatus tick() override {
        std::lock_guard<std::mutex> lock(data_mutex_);
        RCLCPP_INFO(this->get_logger(), "Value DATA ===>: %f", data1);
        
        // Set the output port value
        setOutput("value", data1);
        
        return BT::NodeStatus::SUCCESS;
    }

    static BT::PortsList providedPorts() {
        return {
            BT::OutputPort<float>("value", "Current float value")
        }; 
    }


    private:
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    float data1 = 0.0f;
    std::mutex data_mutex_;  

    void topic_callback(const std_msgs::msg::Float32::SharedPtr _msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        data1 = _msg->data;  // Store the data
        RCLCPP_INFO(this->get_logger(), "I heard: %f", data1);
    }

    void timer_callback() {
        auto message = std_msgs::msg::Float32();
        message.data = data1++;
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Publishing new data: %f", message.data);
    }
};



static const char *xml_text = R"(
 <root>
    <BehaviorTree>
        <Sequence>
            <ReadFloat value="{current_value}"/>
            <FloatThresholdDecorator threshold="0.7" value="{current_value}">
                <AlwaysSuccess/> <!-- or some other action node -->
            </FloatThresholdDecorator>
        </Sequence>
    </BehaviorTree>
</root>
 )";

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
//   auto nh = std::make_shared<rclcpp::Node>("sleep_client");
  BehaviorTreeFactory factory;
  factory.registerNodeType<ReadFloat>("ReadFloat");
  factory.registerNodeType<FloatThresholdDecorator>("FloatThresholdDecorator");

  auto tree = factory.createTreeFromText(xml_text);
  NodeStatus status = NodeStatus::IDLE;
  BT::NodeConfiguration con = {};
  auto lc_listener = std::make_shared<ReadFloat>("lc_listener", con);
  while (true) {
    rclcpp::spin_some(lc_listener);
    status = tree.tickRoot();
    tree.sleep(std::chrono::milliseconds(100));
  }
  return 0;
}
