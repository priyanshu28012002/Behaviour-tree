#include "my_sequence_node.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace BT
{


MySequenceNode::MySequenceNode(const std::string& name)
    : ControlNode::ControlNode(name, {} )
  , current_child_idx_(0)
{
    setRegistrationID("Sequence");
}

void MySequenceNode::halt()
{
    current_child_idx_ = 0;
    ControlNode::halt();
}

NodeStatus MySequenceNode::tick()
{
    const size_t children_count = children_nodes_.size();

    setStatus(NodeStatus::RUNNING);

    while (current_child_idx_ < children_count)
    {
        TreeNode* current_child_node = children_nodes_[current_child_idx_];
        const NodeStatus child_status = current_child_node->executeTick();

        switch (child_status)
        {
            case NodeStatus::RUNNING:
            {
                return child_status;
            }
            case NodeStatus::FAILURE:
            {
                // Reset on failure
                haltChildren();
                current_child_idx_ = 0;
                return child_status;
            }
            case NodeStatus::SUCCESS:
            {
                current_child_idx_++;
            }
            break;

            case NodeStatus::IDLE:
            {
                throw LogicError("A child node must never return IDLE");
            }
        }   // end switch
    }       // end while loop

    // The entire while loop completed. This means that all the children returned SUCCESS.
    if (current_child_idx_ == children_count)
    {
        haltChildren();
        current_child_idx_ = 0;
    }
    return NodeStatus::SUCCESS;
}

}


int main(){
    return 0;
}