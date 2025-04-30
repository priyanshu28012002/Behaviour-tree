#include <iostream>
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp_v3/basic_types.h"
#include "behaviortree_cpp_v3/tree_node.h"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
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
    
            return data1 > 0.5f ? BT::NodeStatus::SUCCESS : BT::NodeStatus::SUCCESS;
        }
    
        static BT::PortsList providedPorts() {
            return {}; 
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



// -------------------------------------------------------------------------------------------------------------------    


#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <atomic>
#include <thread>
#include <optional>

// Node that subscribes to /val and sets the blackboard value
class SetValueNode : public BT::SyncActionNode, public rclcpp::Node
{
public:
  SetValueNode(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config),
      Node("set_value_node")
  {
    subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      "/val", 10,
      [this](const std_msgs::msg::Int32::SharedPtr msg) {
        last_value_ = msg->data;
      });
  }

  static BT::PortsList providedPorts()
  {
    return { BT::OutputPort<int>("Val") };
  }

  BT::NodeStatus tick() override
  {
    if (!last_value_) {

      RCLCPP_WARN(get_logger(), "No value received yet");
      return BT::NodeStatus::FAILURE;
    }
    std::this_thread::sleep_for(std::chrono::seconds(10));
    setOutput("Val", last_value_);
    RCLCPP_INFO(get_logger(), "Setting value: %d", last_value_);
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
  int last_value_;
};

// Node that gets the value from blackboard and publishes to /bt_val
class GetValueNode : public BT::SyncActionNode, public rclcpp::Node
{
public:
  GetValueNode(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config),
      Node("get_value_node")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("/bt_val", 10);
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<int>("Val") };
  }

  BT::NodeStatus tick() override
  {
    auto val = getInput<int>("Val");
    
    if(!val) {
      RCLCPP_ERROR(get_logger(), "Error reading value: %s", val.error().c_str());
      return BT::NodeStatus::FAILURE;
    }
    
    std::this_thread::sleep_for(std::chrono::seconds(10));

    auto message = std_msgs::msg::Int32();
    message.data = val.value();
    publisher_->publish(message);
    
    RCLCPP_INFO(get_logger(), "Published value: %d", val.value());
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  // Create behavior tree factory
  BT::BehaviorTreeFactory factory;
  
  // Register custom nodes
  factory.registerNodeType<SetValueNode>("SetValue");
  factory.registerNodeType<GetValueNode>("GetValue");
  
  // Simple tree: SetValue -> GetValue
  const std::string xml_tree = R"(
  <root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
      <Sequence name="main_sequence">
        <SetValue Val="{Val}"/>
        <GetValue Val="{Val}"/>
      </Sequence>
    </BehaviorTree>
  </root>
  )";
  
  // Create tree from XML string
  auto tree = factory.createTreeFromText(xml_tree);
  
  // Optional: Add logger to monitor the tree in Groot
  // BT::PublisherZMQ publisher_zmq(tree);
  BT::PublisherZMQ publisher_zmq(tree);

  
  // Spin ROS nodes in separate threads
  rclcpp::executors::MultiThreadedExecutor executor;
  
  // Collect node shared pointers
  std::vector<std::shared_ptr<rclcpp::Node>> ros_nodes;
  for(auto& node : tree.nodes)
  {
    if(auto ros_node = dynamic_cast<rclcpp::Node*>(node.get()))
    {
      // Store shared pointer and add to executor
      auto shared_node = std::shared_ptr<rclcpp::Node>(ros_node, [](auto*) {});
      ros_nodes.push_back(shared_node);
      executor.add_node(shared_node);
    }
  }
  
  // Flag for controlling the BT execution thread
  std::atomic<bool> running{true};
  
  // Execute the tree in a separate thread
  std::thread bt_thread([&tree, &running]() {
    while(rclcpp::ok() && running) {
      tree.tickRoot();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  });
  
  // Spin ROS nodes
  executor.spin();
  
  // Signal BT thread to stop
  running = false;
  if(bt_thread.joinable()) {
    bt_thread.join();
  }
  
  rclcpp::shutdown();
  return 0;
}
















------------------------------------------------------------------------------------------------------------------------------------------


#include "patrolling_bot/bt_nodes.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

std::atomic_bool switchActive{true};

static const char *xml_text = R"(
<root>
  <BehaviorTree>
    <Sequence name="MainSequence">
      <Update_BlackBoard "last_battery_status_={last_battery_status_}" />
    </Sequence>
  </BehaviorTree>
</root>
)";

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Create BehaviorTree factory and register custom node
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<patrolling_robot::Update_BlackBoard>("Update_BlackBoard");
  // Optional direct instance (if needed elsewhere)
  BT::NodeConfiguration config = {};
  auto update_blackboard_node = std::make_shared<patrolling_robot::Update_BlackBoard>("Update_BlackBoard", config);

  // Set up the blackboard
  BT::Blackboard::Ptr blackboard = BT::Blackboard::create();
  std_msgs::msg::Float32 battery_status_;
        std_msgs::msg::Float32MultiArray risk_data_;
        nav_msgs::msg::Odometry odom_data_;

  blackboard->set("last_battery_status_",0);
  blackboard->set("last_risk_",risk_data_);
  blackboard->set("last_odom_data_",odom_data_);
    // Create the tree from XML description
    auto tree = factory.createTreeFromText(xml_text, blackboard);
    //BT::StdCoutLogger logger(tree);
  // Tick the tree in a loop
  while (rclcpp::ok()) {
    rclcpp::spin_some(update_blackboard_node);
    tree.tickRoot();
    tree.sleep(100ms);
  }

  rclcpp::shutdown();
  return 0;
}

------------------------------------------------------------------------------------------------------------------------------------------


#include "patrolling_bot/bt_nodes.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"

// Node that sets a random value to the blackboard
class SetRandomValue : public BT::SyncActionNode
{
public:
  SetRandomValue(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return { BT::OutputPort<int>("Val") };
  }

  BT::NodeStatus tick() override
  {
    // Generate random value between 1-100
    int random_val = rand() % 100 + 1;
    
    // Set value to blackboard
    setOutput("Val", random_val);
    
    RCLCPP_INFO(rclcpp::get_logger("SetRandomValue"), "Setting random value: %d", random_val);
    
    return BT::NodeStatus::SUCCESS;
  }
};

// Node that gets the value from the blackboard
class GetValue : public BT::SyncActionNode
{
public:
  GetValue(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<int>("Val") };
  }

  BT::NodeStatus tick() override
  {
    auto val = getInput<int>("Val");
    
    if(!val)
    {
      RCLCPP_ERROR(rclcpp::get_logger("GetValue"), "Error reading value: %s", val.error().c_str());
      return BT::NodeStatus::FAILURE;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("GetValue"), "Got value: %d", val.value());
    
    return BT::NodeStatus::SUCCESS;
  }
};

std::atomic_bool switchActive{true};

static const char *xml_text = R"(
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="main_sequence">
      <SetRandomValue Val="{Val}"/>
      <GetValue Val="{Val}"/>
    </Sequence>
  </BehaviorTree>
</root>
)";

#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  // Create behavior tree factory
  BT::BehaviorTreeFactory factory;
  
  // Register custom nodes
  factory.registerNodeType<SetRandomValue>("SetRandomValue");
  factory.registerNodeType<GetValue>("GetValue");
  
  // Create tree from XML
   auto tree = factory.createTreeFromText(xml_text);

  
  // Optional: Add logger to monitor the tree in Groot
  BT::PublisherZMQ publisher_zmq(tree);
  
  // Execute the tree
  tree.tickRoot();
  
  rclcpp::shutdown();
  return 0;
}


// last working 2 : 17 24 april 
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include <atomic>
#include <optional>
#include <thread>

class UpdateBlackBoardNode : public BT::SyncActionNode
{
public:
    UpdateBlackBoardNode(const std::string& name,
                         const BT::NodeConfiguration& config,
                         const rclcpp::Node::SharedPtr& node)
        : BT::SyncActionNode(name, config),
          node_(node)
    {
        subscription_ = node_->create_subscription<std_msgs::msg::Float32>(
            "/bt/battery_status", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                last_value_ = msg->data;
                RCLCPP_INFO(node_->get_logger(), "Received battery status: %.2f", *last_value_);
            });
    }

    static BT::PortsList providedPorts()
    {
        return { BT::OutputPort<float>("battery_status") };
    }

    BT::NodeStatus tick() override
    {
        if (!last_value_) {
            RCLCPP_WARN(node_->get_logger(), "No battery status received yet.");
            return BT::NodeStatus::FAILURE;
        }

        setOutput("battery_status", *last_value_);
        RCLCPP_INFO(node_->get_logger(), "Set battery status: %.2f", *last_value_);
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
    std::optional<float> last_value_;
};


class PrintBatteryStatus : public BT::SyncActionNode
{
public:
    PrintBatteryStatus(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<float>("battery_status") };
    }

    BT::NodeStatus tick() override
    {
        float value;
        if (!getInput("battery_status", value)) {
            std::cerr << "PrintBatteryStatus: Failed to get battery_status from blackboard\n";
            return BT::NodeStatus::FAILURE;
        }

        std::cout << "[PrintBatteryStatus] Battery level: " << value << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto ros_node = rclcpp::Node::make_shared("bt_main_node");

    BT::BehaviorTreeFactory factory;

    factory.registerBuilder<UpdateBlackBoardNode>(
        "UpdateBlackBoard",
        [ros_node](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<UpdateBlackBoardNode>(name, config, ros_node);
        });

    factory.registerNodeType<PrintBatteryStatus>("PrintBatteryStatus");

    const std::string xml_tree = R"(
    <root main_tree_to_execute="MainTree">
        <BehaviorTree ID="MainTree">
            <Sequence name="main_sequence">
                <UpdateBlackBoard battery_status="{battery_status}"/>
                <PrintBatteryStatus battery_status="{battery_status}"/>
            </Sequence>
        </BehaviorTree>
    </root>
    )";

    auto tree = factory.createTreeFromText(xml_tree);

    BT::PublisherZMQ publisher_zmq(tree);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(ros_node);

    std::atomic<bool> running{true};

    std::thread bt_thread([&tree, &running]() {
        while (rclcpp::ok() && running) {
            tree.tickRoot();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });

    executor.spin();
    running = false;
    if (bt_thread.joinable()) {
        bt_thread.join();
    }

    rclcpp::shutdown();
    return 0;
}
