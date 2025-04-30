#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <atomic>
#include <thread>
#include <optional>
#include <vector>
#include <iostream>

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/condition_node.h>
#include <behaviortree_cpp_v3/behavior_tree.h>

#include <chrono>
#include <string>

class UpdateBlackBoardNode : public BT::SyncActionNode
{
public:
    UpdateBlackBoardNode(const std::string &name,
                         const BT::NodeConfiguration &config,
                         const rclcpp::Node::SharedPtr &node)
        : BT::SyncActionNode(name, config),
          node_(node)
    {
        // Create subscribers
        battery_subscriber_ = node_->create_subscription<std_msgs::msg::Float32>(
            "/bt/battery_status", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg)
            {
                battery_status_ = msg->data;
            });

        risk_subscriber_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/bt/risk", 10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg)
            {
                risk_values_ = msg->data;
            });

        odom_subscriber_ = node_->create_subscription<nav_msgs::msg::Odometry>(
            "/bt/odom", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg)
            {
                position_ = {msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z};
                orientation_ = {msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                                msg->pose.pose.orientation.z, msg->pose.pose.orientation.w};
            });

        gole_pose_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/bt/gole", 10);

        timer_ = node_->create_wall_timer(
            std::chrono::seconds(1),
            [this]()
            {
                if (!getInput("gole_pose", gole_pose_))
                {
                    // std::cerr << "Failed to get gole_pose from blackboard\n";
                    return;
                }

                if (gole_pose_.size() > 0)
                {
                    geometry_msgs::msg::PoseStamped msg;
                    msg.header.stamp = node_->get_clock()->now();
                    msg.header.frame_id = "base_link";
                    msg.pose.position.x = gole_pose_[0];
                    msg.pose.position.y = gole_pose_[1];
                    msg.pose.position.z = gole_pose_[2];
                    msg.pose.orientation.x = gole_pose_[3];
                    msg.pose.orientation.y = gole_pose_[4];
                    msg.pose.orientation.z = gole_pose_[5];
                    msg.pose.orientation.w = gole_pose_[6];

                    gole_pose_publisher_->publish(msg);
                }
            });
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::OutputPort<float>("battery_status"),
            BT::OutputPort<std::vector<float>>("risk"),
            BT::OutputPort<std::vector<double>>("position"),
            BT::OutputPort<std::vector<double>>("orientation"),
            BT::InputPort<std::vector<double>>("gole_pose")};
    }

    BT::NodeStatus tick() override
    {
        bool ok = checkAllDataReceived();

        if (battery_status_)
        {
            setOutput("battery_status", *battery_status_);
        }
        if (risk_values_)
        {
            setOutput("risk", *risk_values_);
        }
        if (position_)
        {
            setOutput("position", *position_);
        }
        if (orientation_)
        {
            setOutput("orientation", *orientation_);
        }

        std::this_thread::sleep_for(std::chrono::seconds(5));
        return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

private:
    bool checkAllDataReceived() const
    {
        return battery_status_.has_value() &&
               risk_values_.has_value() &&
               position_.has_value() &&
               orientation_.has_value();
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr risk_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr gole_pose_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<double> gole_pose_; // To store goal pose

    std::optional<float> battery_status_;
    std::optional<std::vector<float>> risk_values_;
    std::optional<std::vector<double>> position_;
    std::optional<std::vector<double>> orientation_;
};

class IsBatteryCritical : public BT::ConditionNode
{
public:
    IsBatteryCritical(const std::string &name, const BT::NodeConfiguration &config,
                      float threshold = 10.0f)
        : BT::ConditionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<float>("battery_status")};
    }

    BT::NodeStatus tick() override
    {
        battery_level_ = 0.0;
        if (!getInput("battery_status", battery_level_))
        {
            std::cerr << "PrintBatteryStatus: Failed to get battery_status from blackboard\n";
            return BT::NodeStatus::FAILURE;
        }

        if (!battery_level_)
        {
            // RCLCPP_WARN(node_->get_logger(), "No battery data received");
            return BT::NodeStatus::FAILURE;
        }

        if (battery_level_ < threshold)
        {
            // RCLCPP_INFO(node_->get_logger(), "Battery critical: %.1f%%", *battery_level_);
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::FAILURE;
    }

private:
    float threshold;
    float battery_level_;
};

class ReturnToCharging : public BT::SyncActionNode
{
public:
    ReturnToCharging(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
    }

    BT::NodeStatus tick() override
    {

        setOutput("gole_pose", gole_pose_);

        return BT::NodeStatus::SUCCESS;
    }

    static BT::PortsList providedPorts()
    {

        return {
            BT::OutputPort<std::vector<double>>("gole_pose")};
    }

private:
    std::vector<double> gole_pose_;
};

class PatrollTOGole : public BT::SyncActionNode
{
public:
    PatrollTOGole(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {

        // Initialize waypoints (example data)
        checkPoint1 = {1.74, -1.35, 0.0, 0.0, 0.0, 0.7, 0.7}; // Example waypoint data
        checkPoint2 = {0.0, 2.0, 0.0, 0.0, -5.0, 0.99, 0.0}; // Example waypoint data
        checkPoint3 = {-2.0, 0.0, 0.0, 0.0, 0.0, 0.68, -0.70}; // Example waypoint data
    }

    BT::NodeStatus tick() override
    {
        checkpoint_index_ = -1;
        // Retrieve checkpoint index from the blackboard
        if (!getInput("checkpoint_index", checkpoint_index_))
        {
            std::cerr << "[checkpoint_index] Failed to get checkpoint_index from blackboard\n";
            return BT::NodeStatus::FAILURE;
        }

        // Check the value of checkpoint_index_ and set gole_pose_ accordingly
        if (checkpoint_index_ != -1)
        {
            gole_pose_.resize(7); // Resize gole_pose_ to hold position and orientation values

            switch (checkpoint_index_)
            {
            case 0:
                gole_pose_ = checkPoint1;
                break;
            case 1:
                gole_pose_ = checkPoint2;
                break;
            case 2:
                gole_pose_ = checkPoint3;
                break;
            default:
                std::cerr << "Invalid checkpoint index\n";
                return BT::NodeStatus::FAILURE;
            }
        }

        // Set the "gole_pose" output on the blackboard
        setOutput("gole_pose", gole_pose_);

        return BT::NodeStatus::SUCCESS;
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<int>("checkpoint_index"), // Expected type for checkpoint_index is int
            BT::OutputPort<std::vector<double>>("gole_pose")};
    }

private:
    int checkpoint_index_;
    std::vector<double> gole_pose_;
    std::vector<double> checkPoint1;
    std::vector<double> checkPoint2;
    std::vector<double> checkPoint3;
};

class OrderRiskProbability : public BT::SyncActionNode
{
public:
    OrderRiskProbability(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config),
          checkpoint_index_(-1)
    {
    }

    BT::NodeStatus tick() override
    {
        if (!getInput("risk", risk_)) {
            std::cerr << "OrderRiskProbability: Failed to get 'risk' from blackboard\n";
            return BT::NodeStatus::FAILURE;
        }

        if (risk_.empty()) {
            std::cerr << "OrderRiskProbability: 'risk' vector is empty\n";
            return BT::NodeStatus::FAILURE;
        }

        // Find the index of the maximum risk value
        float max_risk = risk_[0];
        checkpoint_index_ = 0;

        for (size_t i = 1; i < risk_.size(); ++i) {
            if (risk_[i] > max_risk) {
                max_risk = risk_[i];
                checkpoint_index_ = static_cast<int>(i);
            }
        }

        setOutput("checkpoint_index", checkpoint_index_);
        std::cout << "OrderRiskProbability: checkpoint_index set to " << checkpoint_index_ << " (max risk = " << max_risk << ")\n";

        return BT::NodeStatus::SUCCESS;
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::OutputPort<int>("checkpoint_index"),
            BT::InputPort<std::vector<float>>("risk")
        };
    }

private:
    int checkpoint_index_;
    std::vector<float> risk_;
};


const std::string xml_tree = R"(
  <root main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree">
          <Sequence name="main_sequence">
              <UpdateBlackBoard battery_status="{battery_status}" risk="{risk}" position="{position}" orientation="{orientation}" gole_pose="{gole_pose}"/>
              <Fallback>
              <Sequence name="EmergencyLowBattery">
              <IsBatteryCritical battery_status="{battery_status}"/>
              <ReturnToCharging gole_pose="{gole_pose}"/>
             </Sequence>
             
             <Fallback/>

              <OrderRiskProbability risk="{risk}" checkpoint_index="{checkpoint_index}"/>
              <PatrollTOGole gole_pose="{gole_pose}" checkpoint_index="{checkpoint_index}"/>
          </Sequence>
      </BehaviorTree>
  </root>
  )";

// ========== Main ==========
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto ros_node = rclcpp::Node::make_shared("bt_main_node");

    BT::BehaviorTreeFactory factory;

    factory.registerBuilder<UpdateBlackBoardNode>(
        "UpdateBlackBoard",
        [ros_node](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<UpdateBlackBoardNode>(name, config, ros_node);
        });
    factory.registerNodeType<IsBatteryCritical>("IsBatteryCritical");
    factory.registerNodeType<ReturnToCharging>("ReturnToCharging");
    factory.registerNodeType<PatrollTOGole>("PatrollTOGole");
    factory.registerNodeType<OrderRiskProbability>("OrderRiskProbability");


    auto tree = factory.createTreeFromText(xml_tree);
    BT::StdCoutLogger logger(tree);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(ros_node);

    std::atomic<bool> running{true};

    std::thread bt_thread([&tree, &running]()
                          {
        while (rclcpp::ok() && running) {
            tree.tickRoot();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } });

    executor.spin();
    running = false;

    if (bt_thread.joinable())
    {
        bt_thread.join();
    }

    rclcpp::shutdown();
    return 0;
}
