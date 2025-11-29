#pragma once

#include <string>

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace mobile_manipulator_tasks
{

class ComputeBaseGoalFromObject : public BT::SyncActionNode
{
public:
  ComputeBaseGoalFromObject(const std::string& name,
                            const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

}  
