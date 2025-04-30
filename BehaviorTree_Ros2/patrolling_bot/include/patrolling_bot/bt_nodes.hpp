#pragma once

#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "behaviortree_cpp_v3/basic_types.h"
#include "behaviortree_cpp_v3/tree_node.h"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace patrolling_robot
{

    class BatteryMonitor : public BT::StatefulActionNode, public rclcpp::Node
    {
    public:
        BatteryMonitor(const std::string &name, const BT::NodeConfiguration &config);

        static BT::PortsList providedPorts();
        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
        void batteryCallback(const std_msgs::msg::Float32::SharedPtr msg);

        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_sub_;
        float battery_level_ = 100.0;
        std::mutex mutex_;
    };

    class SafetyCheck : public BT::ConditionNode, public rclcpp::Node
    {
    public:
        SafetyCheck(const std::string &name, const BT::NodeConfiguration &config);
        BT::NodeStatus tick() override;
        static BT::PortsList providedPorts();

    private:
        void safetyCallback(const std_msgs::msg::Float32::SharedPtr msg);

        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr safety_sub_;
        bool is_safe_ = true;
        std::mutex mutex_;
    };

    class ReturnToCharging : public BT::AsyncActionNode, public rclcpp::Node
    {
    public:
        ReturnToCharging(const std::string &name, const BT::NodeConfiguration &config);
        BT::NodeStatus tick() override;
        static BT::PortsList providedPorts();

    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    };

    class PatrolAction : public BT::AsyncActionNode, public rclcpp::Node
    {
    public:
        PatrolAction(const std::string &name, const BT::NodeConfiguration &config);
        BT::NodeStatus tick() override;
        static BT::PortsList providedPorts();

    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    };

    // class Update_BlackBoard : public BT::AsyncActionNode, public rclcpp::Node
    // {

    // public:
    //     Update_BlackBoard(const std::string &name, const BT::NodeConfiguration &config);
    //         // : BT::AsyncActionNode(name, config), Node("BT_Node") {};

    //     static BT::PortsList providedPorts();

    //     virtual void halt() override;

    //     BT::NodeStatus tick() override;

    // private:
    //     rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_status_subscriber_;

    //     rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr risk_subscriber_;

    //     rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    //     std_msgs::msg::Float32 last_battery_status_;
    //     std_msgs::msg::Float32MultiArray last_risk_data_;
    //     nav_msgs::msg::Odometry last_odom_data_;

    //     void battery_status_subscriber_callback_(const std_msgs::msg::Float32 _msg);
    //     void risk_subscriber_callback_(const std_msgs::msg::Float32MultiArray::SharedPtr &_msg);
    //     void odom_subscriber_callback_(const nav_msgs::msg::Odometry::SharedPtr &_msg);
    // };

//     class Update_BlackBoard : public BT::AsyncActionNode, public rclcpp::Node
// {
// public:
//     Update_BlackBoard(const std::string &name, const BT::NodeConfiguration &config);

//     static BT::PortsList providedPorts();
//     virtual void halt() override;
//     BT::NodeStatus tick() override;

// private:
//     rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr battery_status_subscriber_;
//     std_msgs::msg::Int32 last_battery_status_;
//     std::mutex data_mutex_;

//     void battery_status_subscriber_callback_(const std_msgs::msg::Int32::SharedPtr &msg);
// };

} // namespace patrolling_robot