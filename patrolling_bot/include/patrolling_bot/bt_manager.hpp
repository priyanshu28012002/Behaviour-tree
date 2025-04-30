#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace patrolling_bot {

class BTManager : public rclcpp::Node {
public:
    BTManager();
    void execute();

private:
    void setup_tree();
    void tick_loop();
    
    std::shared_ptr<BT::BehaviorTreeFactory> factory_;
    BT::Tree tree_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace patrolling_bot