#include "patrolling_bot/bt_manager.hpp"
#include "patrolling_bot/bt_nodes.hpp"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"

namespace patrolling_bot {

BTManager::BTManager() 
    : Node("bt_manager") 
{
    // Initialize Behavior Tree
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();
    BT::BehaviorTreeFactory factory;

    // Register custom nodes
    factory.registerNodeType<patrolling_robot::BatteryMonitor>("BatteryMonitor");
    factory.registerNodeType<patrolling_robot::SafetyCheck>("SafetyCheck");
    factory.registerNodeType<patrolling_robot::ReturnToCharging>("ReturnToCharging");
    factory.registerNodeType<patrolling_robot::PatrolAction>("PatrolAction");
    
    // Create tree
    setup_tree();
    
    // Set up timer for ticking
}

void BTManager::setup_tree() {
    static const char* xml_tree = R"(
    <root main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree">
        <Fallback name="Root">
          <Sequence name="EmergencyProtocol">
            <BatteryMonitor min_level="10"/>
            <ReturnToCharging/>
          </Sequence>
          <Sequence name="SafetyProtocol">
            <Invert>
              <SafetyCheck/>
            </Invert>
            <ReturnToCharging/>
          </Sequence>
          <Fallback name="NormalOperation">
            <Sequence name="HighPriorityPatrol">
              <BatteryMonitor min_level="30"/>
              <PatrolAction/>
            </Sequence>
            <Sequence name="StandardPatrol">
              <BatteryMonitor min_level="20"/>
              <PatrolAction/>
            </Sequence>
            <AlwaysSuccess/>
          </Fallback>
        </Fallback>
      </BehaviorTree>
    </root>
    )";
    
    tree_ = factory_->createTreeFromText(xml_tree);
    
    // Add logger
    static BT::StdCoutLogger logger(tree_);
}

void BTManager::tick_loop() {
    // Tick the behavior tree
    tree_.tickRoot();
    
    // Process any ROS callbacks
    rclcpp::spin_some(shared_from_this());
}

void BTManager::execute() {
    rclcpp::spin(shared_from_this());
}

} // namespace patrolling_bot