#include "patrolling_bot/bt_nodes.hpp"

namespace patrolling_robot
{
    BatteryMonitor::BatteryMonitor(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config), Node("battery_monitor")
    {
        battery_sub_ = create_subscription<std_msgs::msg::Float32>(
            "/battery_level", 10, std::bind(&BatteryMonitor::batteryCallback, this, std::placeholders::_1));
    }

    BT::PortsList BatteryMonitor::providedPorts()
    {
        return {BT::InputPort<float>("min_level")};
    }

    BT::NodeStatus BatteryMonitor::onStart()
    {
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus BatteryMonitor::onRunning()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        float min_level;
        if (!getInput("min_level", min_level))
        {
            throw BT::RuntimeError("missing required input [min_level]");
        }
        return (battery_level_ >= min_level) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    void BatteryMonitor::onHalted()
    {
        RCLCPP_INFO(get_logger(), "Battery monitoring halted");
    }

    void BatteryMonitor::batteryCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        battery_level_ = msg->data;
    }

    SafetyCheck::SafetyCheck(const std::string &name, const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config), Node("safety_check")
    {
        safety_sub_ = create_subscription<std_msgs::msg::Float32>(
            "/safety_status", 10, std::bind(&SafetyCheck::safetyCallback, this, std::placeholders::_1));
    }

    BT::NodeStatus SafetyCheck::tick()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return is_safe_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::PortsList SafetyCheck::providedPorts()
    {
        return {};
    }

    void SafetyCheck::safetyCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        is_safe_ = (msg->data > 0.5f);
    }

    ReturnToCharging::ReturnToCharging(const std::string &name, const BT::NodeConfiguration &config)
        : BT::AsyncActionNode(name, config), Node("return_to_charging")
    {
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

    BT::NodeStatus ReturnToCharging::tick()
    {
        RCLCPP_INFO(get_logger(), "Returning to charging station");

        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = 0.5;
        cmd_vel_pub_->publish(twist);

        std::this_thread::sleep_for(std::chrono::seconds(3));
        return BT::NodeStatus::SUCCESS;
    }

    BT::PortsList ReturnToCharging::providedPorts()
    {
        return {};
    }

    PatrolAction::PatrolAction(const std::string &name, const BT::NodeConfiguration &config)
        : BT::AsyncActionNode(name, config), Node("patrol_action")
    {
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

    BT::NodeStatus PatrolAction::tick()
    {
        RCLCPP_INFO(get_logger(), "Executing patrol pattern");

        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = 0.3;
        twist.angular.z = 0.1;
        cmd_vel_pub_->publish(twist);

        std::this_thread::sleep_for(std::chrono::seconds(3));
        return BT::NodeStatus::SUCCESS;
    }

    BT::PortsList PatrolAction::providedPorts()
    {
        return {};
    }

    // Update_BlackBoard::Update_BlackBoard(const std::string &name, const BT::NodeConfiguration &config)
    //     : BT::AsyncActionNode(name, config), Node("BT_Node")
    // {

    //     auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
    //     battery_status_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
    //         "/bt/battery_status", sensor_qos,
    //         [&](const std_msgs::msg::Float32::SharedPtr _msg)
    //         {
    //             battery_status_subscriber_callback_(_msg);
    //         });

    //     risk_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
    //         "/bt/risk", sensor_qos,
    //         [&](const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    //         {
    //             risk_subscriber_callback_(msg);
    //         });
    //     odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
    //         "/bt/odom", sensor_qos,
    //         [&](const nav_msgs::msg::Odometry::SharedPtr msg)
    //         {
    //             odom_subscriber_callback_(msg);
    //         });

    //     publisher_ = this->create_publisher<std_msgs::msg::Float32>("published_value", 10);

    //     timer_ = this->create_wall_timer(
    //         std::chrono::milliseconds(1000), std::bind(&Update_BlackBoard::timer_callback, this));
    //     int data_ = 10;
    // }

    // BT::NodeStatus Update_BlackBoard::tick()
    // {
    //     RCLCPP_INFO(this->get_logger(), "batery value  set  %f", lastData);

    //     config().blackboard->set("last_battery_status_", lastData);
    //     config().blackboard->set("last_risk_", last_risk_data_);
    //     config().blackboard->set("last_odom_data_", last_odom_data_);

    //     std::this_thread::sleep_for(std::chrono::seconds(1));
    //     std_msgs::msg::Float32 battery_status_;
    //     std_msgs::msg::Float32MultiArray risk_data_;
    //     nav_msgs::msg::Odometry odom_data_;

    //     int res = !config().blackboard->get("last_battery_status_", lastData);
    //     if (res)
    //     {
    //         RCLCPP_INFO(this->get_logger(), "Not get batery value");
    //     }
    //     else{
    //         RCLCPP_INFO(this->get_logger(), "batery value  get  %f", lastData);
    //     }

    //     // if (!config().blackboard->get("last_risk_", risk_data_))
    //     // {
    //     //     RCLCPP_INFO(this->get_logger(), "Not get risk value");
    //     // }

    //     // if (!config().blackboard->get("last_odom_data_", odom_data_))
    //     // {
    //     //     RCLCPP_INFO(this->get_logger(), "Not get odom value");
    //     // }

    //     // int battery_level;
    //     // if (!config().blackboard->get("node", battery_level))
    //     // {
    //     //     battery_level = 100;
    //     // }
    //     // // std::lock_guard<std::mutex> lock(data_mutex_);
    //     // RCLCPP_INFO(this->get_logger(), "Value DATA : %f", battery_level);
    //     // data1++;
    //     return  BT::NodeStatus::SUCCESS ;
    // }
    // void Update_BlackBoard::halt()
    // {
    //    // RCLCPP_INFO(this->get_logger(), "HALT IS CALLING ");
    // }

    // BT::PortsList Update_BlackBoard::providedPorts()
    // {
    //     return {};//BT::OutputPort<int>("output_value")
    // }

    // void Update_BlackBoard::battery_status_subscriber_callback_(const std_msgs::msg::Float32 _msg)
    // {
    //     lastData = _msg.data;
    //     // last_battery_status_ = *_msg;
    //     RCLCPP_INFO(this->get_logger(), "batery value  sub%f", lastData);

    // }
    // void Update_BlackBoard::risk_subscriber_callback_(const std_msgs::msg::Float32MultiArray::SharedPtr &_msg){
    //     last_risk_data_ = *_msg;
    // }
    // void Update_BlackBoard::odom_subscriber_callback_(const nav_msgs::msg::Odometry::SharedPtr &_msg){
    //     last_odom_data_ = *_msg;
    // }

    // void Update_BlackBoard::timer_callback()
    // {
    //     auto message = std_msgs::msg::Float32();

    //     publisher_->publish(message);
    //     //RCLCPP_INFO(this->get_logger(), "Publishing new data: %f", message.data);
    // }

//     Update_BlackBoard::Update_BlackBoard(const std::string &name, const BT::NodeConfiguration &config)
//     : BT::AsyncActionNode(name, config),
//       Node("blackboard_updater_node")
// {
//     last_battery_status_.data = 0; // Initialize with 0

//     // battery_status_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
//     //     "/battery_status",
//     //     rclcpp::QoS(10),
//     //     [this](const std_msgs::msg::Int32::SharedPtr msg) {
//     //         this->battery_status_subscriber_callback_(msg);
//     //     });
// }

// BT::PortsList Update_BlackBoard::providedPorts()
// {
//     return { BT::OutputPort<int>("battery_level") };
// }

// void Update_BlackBoard::halt()
// {
//     RCLCPP_INFO(this->get_logger(), "Update_BlackBoard halted");
// }

// BT::NodeStatus Update_BlackBoard::tick()
// {
//     // std::lock_guard<std::mutex> lock(data_mutex_);
    
//     // // Set the actual subscribed value to blackboard
//     // setOutput("battery_level", last_battery_status_.data);
    
//     // // Read back from blackboard to verify
//     // auto battery_val = getInput<int>("battery_level");
//     // if (!battery_val) {
//     //     RCLCPP_ERROR(get_logger(), "Failed to get battery_level: %s", battery_val.error().c_str());
//     //     return BT::NodeStatus::FAILURE;
//     // }
    
//     // RCLCPP_INFO(get_logger(), "Current battery level: %d", battery_val.value());
//     return BT::NodeStatus::SUCCESS;
// }

// void Update_BlackBoard::battery_status_subscriber_callback_(const std_msgs::msg::Int32::SharedPtr &msg)
// {
//     std::lock_guard<std::mutex> lock(data_mutex_);
//     last_battery_status_ = *msg;
//     RCLCPP_DEBUG(get_logger(), "Received battery update: %d", msg->data);
// }

} // namespace patrolling_robot