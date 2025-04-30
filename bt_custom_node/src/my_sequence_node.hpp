
#ifndef MY_SEQUENCENODE_H
#define MY_SEQUENCENODE_H

#include "behaviortree_cpp_v3/control_node.h"

namespace BT
{
    
class MySequenceNode : public ControlNode
{
  public:
    MySequenceNode(const std::string& name);

    virtual ~MySequenceNode() override = default;

    virtual void halt() override;

  private:
    size_t current_child_idx_;

    virtual BT::NodeStatus tick() override;
};

}

#endif // MY_SEQUENCENODE_H