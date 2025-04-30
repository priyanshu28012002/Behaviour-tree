#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"
#include <chrono>
#include <cmath>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <thread>

class MyBTNode : public BT::SyncActionNode
{
public:
    MyBTNode(const std::string &name) : BT::SyncActionNode(name, {}) {}

    BT::NodeStatus tick() override
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node Tick: Executing MyBTNode");
        return BT::NodeStatus::SUCCESS;
    }
};

class BtNode : public rclcpp::Node {
public:
    BtNode() : Node("battery_status_publisher")
    {
        battery_status_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "battery_status", 10, std::bind(&BtNode::battery_status_callback, this, std::placeholders::_1));  
        setup_bt();  
    }

private:
void battery_status_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Battery status updated: ");
        battery_status_val = *msg;
    }
void setup_bt(){
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<MyBTNode>("MyBTNode");
    BT::Tree tree = factory.createTreeFromFile("src/patrolling_bot/bt_tree/bt_patrolling_bot.xml");
    while (rclcpp::ok())
    {
        tree.tickRoot();
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

}    
rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_status_sub_;
std_msgs::msg::Float32 battery_status_val;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BtNode>());
    rclcpp::shutdown();
    return 0;
}



