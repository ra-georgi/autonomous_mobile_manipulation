#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>

#include "mobile_manipulator_tasks/srv/get_next_object.hpp"
#include "mobile_manipulator_tasks/msg/spawned_object.hpp"

namespace mobile_manipulator_tasks
{

class GetNextObjectBTNode : public BT::SyncActionNode
{
public:
  GetNextObjectBTNode(
      const std::string& name,
      const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<mobile_manipulator_tasks::srv::GetNextObject>::SharedPtr client_;
};

}  
