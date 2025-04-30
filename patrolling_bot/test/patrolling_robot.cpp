// #include "patrolling_bot/bt_nodes.hpp"
// #include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
// #include "behaviortree_cpp_v3/loggers/bt_file_logger.h"

// static const char* xml_tree = R"(
// <root main_tree_to_execute="MainTree">
//   <BehaviorTree ID="MainTree">
//     <Fallback name="Root">
//       <!-- Emergency -->
//       <Sequence name="EmergencyProtocol">
//         <BatteryMonitor min_level="10"/>
//         <ReturnToCharging/>
//       </Sequence>
      
//       <!-- Safety -->
//       <Sequence name="SafetyProtocol">
//         <Invert>
//           <SafetyCheck/>
//         </Invert>
//         <ReturnToCharging/>
//       </Sequence>
      
//       <!-- Normal Operation -->
//       <Fallback name="NormalOperation">
//         <!-- High Priority Patrol -->
//         <Sequence name="HighPriorityPatrol">
//           <BatteryMonitor min_level="30"/>
//           <PatrolAction/>
//         </Sequence>
        
//         <!-- Standard Patrol -->
//         <Sequence name="StandardPatrol">
//           <BatteryMonitor min_level="20"/>
//           <PatrolAction/>
//         </Sequence>
        
//         <!-- Idle -->
//         <AlwaysSuccess/>
//       </Fallback>
//     </Fallback>
//   </BehaviorTree>
// </root>
// )";

// int main(int argc, char** argv)
// {
//     rclcpp::init(argc, argv);
    
//     BT::BehaviorTreeFactory factory;
    
//     // Register custom nodes
//     factory.registerNodeType<patrolling_robot::BatteryMonitor>("BatteryMonitor");
//     factory.registerNodeType<patrolling_robot::SafetyCheck>("SafetyCheck");
//     factory.registerNodeType<patrolling_robot::ReturnToCharging>("ReturnToCharging");
//     factory.registerNodeType<patrolling_robot::PatrolAction>("PatrolAction");
    
//     // Create tree
//     auto tree = factory.createTreeFromText(xml_tree);
//     BT::StdCoutLogger logger(tree);
    
//     // Optional: log to file
//     BT::FileLogger file_logger(tree, "bt_trace.fbl");
    
//     rclcpp::Rate rate(10);
//     while (rclcpp::ok()) {
//         tree.tickRoot();
//         rclcpp::spin_some(tree.rootNode());
//         rate.sleep();
//     }
    
//     rclcpp::shutdown();
//     return 0;
// }

#include "patrolling_bot/bt_manager.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto bt_manager = std::make_shared<patrolling_bot::BTManager>();
    bt_manager->execute();
    
    rclcpp::shutdown();
    return 0;
}