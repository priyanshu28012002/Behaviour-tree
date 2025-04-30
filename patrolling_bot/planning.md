Here's a complete ROS2 Humble implementation of your patrolling bot using Behavior Trees with proper package structure and modern ROS2 practices:

### 1. First, create a ROS2 package:
```bash
ros2 pkg create --build-type ament_cmake patrolling_robot --dependencies rclcpp behaviortree_cpp_v3 geometry_msgs sensor_msgs std_msgs
```

### 2. Here's the complete implementation:

#### `include/patrolling_robot/bt_nodes.hpp`
```cpp
#pragma once

#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"
#include <mutex>

namespace patrolling_robot
{

class BatteryMonitor : public BT::StatefulActionNode, public rclcpp::Node
{
public:
    BatteryMonitor(const std::string & name, const BT::NodeConfiguration & config);
    
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
    SafetyCheck(const std::string & name, const BT::NodeConfiguration & config);
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
    ReturnToCharging(const std::string & name, const BT::NodeConfiguration & config);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

class PatrolAction : public BT::AsyncActionNode, public rclcpp::Node
{
public:
    PatrolAction(const std::string & name, const BT::NodeConfiguration & config);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

} // namespace patrolling_robot
```

#### `src/bt_nodes.cpp`
```cpp
#include "patrolling_robot/bt_nodes.hpp"

namespace patrolling_robot
{

BatteryMonitor::BatteryMonitor(const std::string & name, const BT::NodeConfiguration & config)
    : BT::StatefulActionNode(name, config), Node("battery_monitor")
{
    battery_sub_ = create_subscription<std_msgs::msg::Float32>(
        "/battery_level", 10, std::bind(&BatteryMonitor::batteryCallback, this, std::placeholders::_1));
}

BT::PortsList BatteryMonitor::providedPorts()
{
    return { BT::InputPort<float>("min_level") };
}

BT::NodeStatus BatteryMonitor::onStart()
{
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus BatteryMonitor::onRunning()
{
    std::lock_guard<std::mutex> lock(mutex_);
    float min_level;
    if (!getInput("min_level", min_level)) {
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

SafetyCheck::SafetyCheck(const std::string & name, const BT::NodeConfiguration & config)
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

ReturnToCharging::ReturnToCharging(const std::string & name, const BT::NodeConfiguration & config)
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
    
    std::this_thread::sleep_for(3s);
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList ReturnToCharging::providedPorts()
{
    return {};
}

PatrolAction::PatrolAction(const std::string & name, const BT::NodeConfiguration & config)
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
    
    std::this_thread::sleep_for(5s);
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList PatrolAction::providedPorts()
{
    return {};
}

} // namespace patrolling_robot
```

#### `src/patrolling_robot.cpp`
```cpp
#include "patrolling_robot/bt_nodes.hpp"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"

static const char* xml_tree = R"(
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Fallback name="Root">
      <!-- Emergency -->
      <Sequence name="EmergencyProtocol">
        <BatteryMonitor min_level="10"/>
        <ReturnToCharging/>
      </Sequence>
      
      <!-- Safety -->
      <Sequence name="SafetyProtocol">
        <Invert>
          <SafetyCheck/>
        </Invert>
        <ReturnToCharging/>
      </Sequence>
      
      <!-- Normal Operation -->
      <Fallback name="NormalOperation">
        <!-- High Priority Patrol -->
        <Sequence name="HighPriorityPatrol">
          <BatteryMonitor min_level="30"/>
          <PatrolAction/>
        </Sequence>
        
        <!-- Standard Patrol -->
        <Sequence name="StandardPatrol">
          <BatteryMonitor min_level="20"/>
          <PatrolAction/>
        </Sequence>
        
        <!-- Idle -->
        <AlwaysSuccess/>
      </Fallback>
    </Fallback>
  </BehaviorTree>
</root>
)";

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    BT::BehaviorTreeFactory factory;
    
    // Register custom nodes
    factory.registerNodeType<patrolling_robot::BatteryMonitor>("BatteryMonitor");
    factory.registerNodeType<patrolling_robot::SafetyCheck>("SafetyCheck");
    factory.registerNodeType<patrolling_robot::ReturnToCharging>("ReturnToCharging");
    factory.registerNodeType<patrolling_robot::PatrolAction>("PatrolAction");
    
    // Create tree
    auto tree = factory.createTreeFromText(xml_tree);
    BT::StdCoutLogger logger(tree);
    
    // Optional: log to file
    BT::FileLogger file_logger(tree, "bt_trace.fbl");
    
    rclcpp::Rate rate(10);
    while (rclcpp::ok()) {
        tree.tickRoot();
        rclcpp::spin_some(tree.rootNode());
        rate.sleep();
    }
    
    rclcpp::shutdown();
    return 0;
}
```

#### `CMakeLists.txt`
```cmake
cmake_minimum_required(VERSION 3.8)
project(patrolling_robot)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(bt_nodes src/bt_nodes.cpp)
target_include_directories(bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(bt_nodes
  rclcpp
  behaviortree_cpp_v3
  geometry_msgs
  std_msgs)

add_executable(patrolling_robot src/patrolling_robot.cpp)
target_link_libraries(patrolling_robot bt_nodes)
ament_target_dependencies(patrolling_robot
  rclcpp
  behaviortree_cpp_v3
  geometry_msgs
  std_msgs)

install(TARGETS
  bt_nodes
  patrolling_robot
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

ament_package()
```

#### `package.xml`
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>patrolling_robot</name>
  <version>0.0.0</version>
  <description>ROS2 Humble patrolling robot with Behavior Trees</description>

  <maintainer email="you@email.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>behaviortree_cpp_v3</depend>
  <depend>geometry_msgs</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### Key Features for ROS2 Humble:
1. **Proper ROS2 Node Integration**: Each BT node is also a ROS2 node
2. **Thread Safety**: Mutex-protected shared data
3. **Modern C++**: Uses C++17 features
4. **Clean Package Structure**: Separation of headers and implementation
5. **Behavior Tree Logging**: Both console and file logging
6. **Configurable**: Battery thresholds and behaviors can be adjusted via parameters

To build and run:
```bash
colcon build --packages-select patrolling_robot
source install/setup.bash
ros2 run patrolling_robot patrolling_robot
```

For a complete system, you'll want to add:
1. Launch files
2. Parameter configuration
3. RViz visualization
4. Actual sensor drivers
5. Navigation stack integration (Nav2)

Would you like me to expand on any particular aspect of this implementation?

=============================================================
1 Node Make the node that lode the update the black box

list of pub and sub required in BT patrolling 

3d array carry the risk percentage 
float battery 
odom